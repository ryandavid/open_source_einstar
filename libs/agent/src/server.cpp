#include "einstar/agent/server.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <format>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "einstar/core/log.hpp"

namespace einstar::agent {

namespace {
constexpr int kSettleFrames = 2;         // a mutating method answers after this many more frames
constexpr int kDefaultTimeoutMs = 15000;  // how long a caller waits for the UI thread
}  // namespace

struct Server::Job {
    std::string line;
    std::promise<std::string> reply;
    json id;
    const MethodSpec* spec = nullptr;
    Poll poll;
    std::optional<Result> result;
    std::uint64_t release_at = 0;
};

Server::Server(App app, std::string name) : app_(app), name_(std::move(name)) {
    log::set_sink([this](log::Level l, std::string_view m) { logs_.add(l, m); });
    register_builtins();
}

Server::~Server() {
    stop();
    log::set_sink({});
}

void Server::handle(std::string_view method, Handler handler) {
    const MethodSpec* s = find_spec(method);
    if (!s || (s->app != App::any && s->app != app_))
        throw std::logic_error(std::format("agent method '{}' has no spec for the {} app", method, app_name(app_)));
    handlers_[std::string(method)] = std::move(handler);
}

// ---- dispatch (UI thread) ----

std::future<std::string> Server::submit(std::string line) {
    auto job = std::make_shared<Job>();
    job->line = std::move(line);
    auto f = job->reply.get_future();
    std::lock_guard lock(mutex_);
    incoming_.push_back(std::move(job));
    return f;
}

void Server::pump() {
    ++frames_;
    // Launched by einstar-mcp: when it goes away (the app is re-parented), nobody can drive or quit us.
    if (parent_pid_ > 1 && ::getppid() != parent_pid_) {
        log::warn("agent: the launching process ({}) is gone: quitting", parent_pid_);
        quit_ = true;
    }
    inspector_.end_frame();
    input_.step();
    std::vector<std::shared_ptr<Job>> fresh;
    {
        std::lock_guard lock(mutex_);
        fresh.swap(incoming_);
    }
    for (const auto& j : fresh) start_job(j);
    // Polls: once per frame until they answer.
    for (auto it = active_.begin(); it != active_.end();) {
        std::optional<Result> r;
        try {
            r = (*it)->poll();
        } catch (const BadParams& e) {
            r = error(ErrorCode::invalid_params, e.what());
        } catch (const std::exception& e) {
            r = error(ErrorCode::internal, e.what());
        }
        if (!r) {
            ++it;
            continue;
        }
        auto job = *it;
        it = active_.erase(it);
        finish(job, std::move(*r));
    }
    for (auto it = settling_.begin(); it != settling_.end();) {
        if (frames_ < (*it)->release_at) {
            ++it;
            continue;
        }
        (*it)->reply.set_value(response_line((*it)->id, *(*it)->result));
        it = settling_.erase(it);
    }
}

void Server::start_job(const std::shared_ptr<Job>& job) {
    json req = json::parse(job->line, nullptr, false);
    if (req.is_discarded()) return job->reply.set_value(response_line(nullptr, error(ErrorCode::parse, "not JSON")));
    if (!req.is_object() || !req.contains("method") || !req["method"].is_string())
        return job->reply.set_value(response_line(req.is_object() ? req.value("id", json()) : json(), error(ErrorCode::invalid_request, "no method")));
    job->id = req.value("id", json());
    const std::string method = req["method"];
    json params = req.value("params", json::object());
    if (params.is_null()) params = json::object();
    if (!params.is_object()) return finish(job, error(ErrorCode::invalid_params, "params must be an object"));
    params.erase("_timeout_ms");
    job->spec = find_spec(method);
    const auto h = handlers_.find(method);
    if (h == handlers_.end()) {
        std::string why = std::format("the {} app has no method '{}'", name_, method);
        if (job->spec) why += std::format(" (the {} app answers it)", app_name(job->spec->app));
        return finish(job, error(ErrorCode::method_not_found, why));
    }
    Outcome out;
    try {
        out = h->second(params);
    } catch (const BadParams& e) {
        out = error(ErrorCode::invalid_params, std::format("{}: {}", method, e.what()));
    } catch (const std::exception& e) {
        out = error(ErrorCode::internal, std::format("{}: {}", method, e.what()));
    }
    if (auto* p = std::get_if<Poll>(&out)) {
        job->poll = std::move(*p);
        active_.push_back(job);
    } else if (auto* j = std::get_if<json>(&out)) {
        finish(job, std::move(*j));
    } else {
        finish(job, std::get<Error>(std::move(out)));
    }
}

void Server::finish(const std::shared_ptr<Job>& job, Result result) {
    // A mutating method that succeeded answers after a couple of frames have shown its effect.
    if (job->spec && job->spec->kind == Kind::mutating && std::holds_alternative<json>(result)) {
        job->result = std::move(result);
        job->release_at = frames_ + kSettleFrames;
        settling_.push_back(job);
        return;
    }
    job->reply.set_value(response_line(job->id, result));
}

Server::Poll Server::after_input(std::uint64_t until, json result) {
    return [this, until, answer = std::move(result)]() -> std::optional<Result> {
        if (!input_.done(until)) return std::nullopt;
        return answer;
    };
}

void Server::provide_capture(Frame frame) {
    capture_ = std::move(frame);
    capture_frame_ = frames_;
    capture_wanted_ = false;
}

std::optional<ImVec2> Server::point_of(const json& p) {
    const auto x = opt_param<float>(p, "x"), y = opt_param<float>(p, "y");
    if (const auto t = opt_param<std::string>(p, "target")) {
        auto r = inspector_.resolve(*t);
        if (auto* e = std::get_if<Error>(&r)) throw BadParams(e->message + (e->data.is_null() ? "" : " " + e->data.dump()));
        const UiItem* it = std::get<const UiItem*>(r);
        if (x && y) return ImVec2(it->min.x + *x, it->min.y + *y);
        return ImVec2((it->min.x + it->max.x) * 0.5f, (it->min.y + it->max.y) * 0.5f);
    }
    if (x && y) return ImVec2(*x, *y);
    return std::nullopt;
}

// ---- the generic methods ----

void Server::register_builtins() {
    handle("rpc.methods", [this](const json&) -> Outcome {
        json names = json::array();
        for (const auto& [n, h] : handlers_) names.push_back(n);
        return json{{"app", name_}, {"methods", names}};
    });
    handle("app.info", [this](const json&) -> Outcome {
        const ImGuiIO& io = ImGui::GetIO();
        return json{{"app", name_},
                    {"kind", app_name(app_)},
                    {"pid", ::getpid()},
                    {"frames", frames_},
                    {"window_points", {io.DisplaySize.x, io.DisplaySize.y}},
                    {"scale", io.DisplayFramebufferScale.x},
                    {"visible", visible_}};
    });
    handle("app.logs", [this](const json& p) -> Outcome {
        const auto lvl_name = opt_param<std::string>(p, "level").value_or("trace");
        log::Level lvl = log::Level::trace;
        for (const auto l : {log::Level::trace, log::Level::debug, log::Level::info, log::Level::warn, log::Level::error})
            if (log::level_name(l) == lvl_name) lvl = l;
        const auto limit = static_cast<std::size_t>(std::clamp(opt_param<int>(p, "limit").value_or(200), 1, 5000));
        return logs_.read(opt_param<std::uint64_t>(p, "since_seq").value_or(0), limit, lvl, opt_param<std::string>(p, "grep").value_or(""));
    });
    handle("app.quit", [this](const json&) -> Outcome {
        quit_ = true;
        return json{{"quitting", true}};
    });
    handle("frame.wait", [this](const json& p) -> Outcome {
        const auto until = frames_ + static_cast<std::uint64_t>(std::clamp(opt_param<int>(p, "frames").value_or(1), 1, 600));
        return Poll([this, until]() -> std::optional<Result> {
            if (frames_ < until) return std::nullopt;
            return json{{"frames", frames_}};
        });
    });
    handle("ui.snapshot", [this](const json& p) -> Outcome {
        const auto win = opt_param<std::string>(p, "window");
        const bool interactive = opt_param<bool>(p, "interactive_only").value_or(false);
        json windows = json::array();
        for (const auto& w : inspector_.windows())
            if (!win || w.name == *win)
                windows.push_back({{"name", w.name}, {"rect", {w.pos.x, w.pos.y, w.size.x, w.size.y}}, {"collapsed", w.collapsed}, {"focused", w.focused}});
        json items = json::array();
        for (const auto& it : inspector_.items()) {
            if (win && it.window != *win) continue;
            if (interactive && (it.label.empty() || it.disabled)) continue;
            items.push_back(it.to_json());
        }
        return json{{"frame", frames_}, {"windows", windows}, {"items", items}};
    });
    handle("ui.find", [this](const json& p) -> Outcome {
        auto r = inspector_.resolve(param<std::string>(p, "target"));
        if (auto* e = std::get_if<Error>(&r)) return *e;
        return std::get<const UiItem*>(r)->to_json();
    });
    handle("ui.screenshot", [this](const json& p) -> Outcome {
        std::optional<std::array<float, 4>> rect;
        if (const auto t = opt_param<std::string>(p, "target")) {
            auto r = inspector_.resolve(*t);
            if (auto* e = std::get_if<Error>(&r)) return *e;
            const UiItem* it = std::get<const UiItem*>(r);
            rect = std::array<float, 4>{it->min.x, it->min.y, it->max.x - it->min.x, it->max.y - it->min.y};
        } else if (const auto r = opt_param<std::vector<float>>(p, "rect")) {
            if (r->size() != 4) throw BadParams("rect is [x, y, w, h]");
            rect = std::array<float, 4>{(*r)[0], (*r)[1], (*r)[2], (*r)[3]};
        }
        const int max_size = opt_param<int>(p, "max_size").value_or(1600);
        capture_wanted_ = true;
        const std::uint64_t asked = frames_;
        return Poll([this, rect, max_size, asked]() -> std::optional<Result> {
            if (!capture_ || capture_frame_ < asked) {
                capture_wanted_ = true;
                return std::nullopt;
            }
            const auto png = encode_png(*capture_, rect ? rect->data() : nullptr, max_size);
            if (!png) return error(ErrorCode::refused, "nothing to capture there (an empty rect?)");
            return json{{"png_base64", base64(png->bytes)},
                        {"scale", png->scale},
                        {"logical_rect", {png->rect[0], png->rect[1], png->rect[2], png->rect[3]}},
                        {"image_size", {png->width, png->height}}};
        });
    });
    handle("ui.wait_for", [this](const json& p) -> Outcome {
        const auto target = param<std::string>(p, "target");
        const auto state = opt_param<std::string>(p, "state").value_or("exists");
        if (state != "exists" && state != "gone" && state != "enabled") throw BadParams("state is exists, gone or enabled");
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(opt_param<int>(p, "timeout_ms").value_or(5000));
        return Poll([this, target, state, deadline]() -> std::optional<Result> {
            auto r = inspector_.resolve(target);
            const auto* it = std::get_if<const UiItem*>(&r);
            const bool met = state == "gone" ? (!it && std::get<Error>(r).code == ErrorCode::not_found)
                                             : (it && (state == "exists" || !(*it)->disabled));
            if (met) return it ? (*it)->to_json() : json{{"gone", true}};
            if (std::chrono::steady_clock::now() > deadline)
                return error(ErrorCode::timeout, std::format("'{}' is not {} yet", target, state), it ? (*it)->to_json() : json());
            return std::nullopt;
        });
    });

    auto button_of = [](const json& p) {
        const auto b = opt_param<std::string>(p, "button").value_or("left");
        if (b == "left") return Button::left;
        if (b == "right") return Button::right;
        if (b == "middle") return Button::middle;
        throw BadParams("button is left, right or middle");
    };
    auto modifiers_of = [](const json& p) {
        try {
            return InputPlayer::parse_modifiers(p.value("modifiers", json()));
        } catch (const std::invalid_argument& e) {
            throw BadParams(e.what());
        }
    };
    handle("input.click", [this, button_of, modifiers_of](const json& p) -> Outcome {
        const auto at = point_of(p);
        if (!at) throw BadParams("give a target, or x and y");
        const auto until = input_.click(*at, button_of(p), opt_param<bool>(p, "double").value_or(false), modifiers_of(p));
        return after_input(until, {{"at", {at->x, at->y}}});
    });
    handle("input.move", [this](const json& p) -> Outcome {
        const ImVec2 at(param<float>(p, "x"), param<float>(p, "y"));
        return after_input(input_.move(at), {{"at", {at.x, at.y}}});
    });
    handle("input.drag", [this, button_of, modifiers_of](const json& p) -> Outcome {
        std::vector<ImVec2> path;
        if (const auto pts = opt_param<std::vector<std::vector<float>>>(p, "path")) {
            for (const auto& q : *pts) {
                if (q.size() != 2) throw BadParams("path points are [x, y]");
                path.emplace_back(q[0], q[1]);
            }
        } else {
            const auto a = param<std::vector<float>>(p, "from"), b = param<std::vector<float>>(p, "to");
            if (a.size() != 2 || b.size() != 2) throw BadParams("from and to are [x, y]");
            const int steps = std::clamp(opt_param<int>(p, "steps").value_or(12), 1, 500);
            for (int i = 0; i <= steps; ++i) {
                const float t = static_cast<float>(i) / static_cast<float>(steps);
                path.emplace_back(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t);
            }
        }
        if (path.size() < 2) throw BadParams("a drag needs at least two points");
        return after_input(input_.drag(path, button_of(p), modifiers_of(p)), {{"points", path.size()}});
    });
    handle("input.scroll", [this](const json& p) -> Outcome {
        std::optional<ImVec2> at;
        if (p.contains("x") && p.contains("y")) at = ImVec2(param<float>(p, "x"), param<float>(p, "y"));
        return after_input(input_.scroll(at, opt_param<float>(p, "dx").value_or(0), opt_param<float>(p, "dy").value_or(0)));
    });
    handle("input.key", [this, modifiers_of](const json& p) -> Outcome {
        const auto name = param<std::string>(p, "key");
        const ImGuiKey k = InputPlayer::key_by_name(name);
        if (k == ImGuiKey_None) throw BadParams(std::format("no key named '{}' (ImGui key names: Escape, Delete, Z, F1, ...)", name));
        return after_input(input_.key(k, modifiers_of(p)));
    });
    handle("input.type", [this](const json& p) -> Outcome { return after_input(input_.type(param<std::string>(p, "text"))); });
}

// ---- the socket ----

bool Server::start(const std::string& path, std::string& why, bool quit_with_parent) {
    std::signal(SIGPIPE, SIG_IGN);  // a client going away must not kill the app
    sockaddr_un addr{};
    if (path.size() >= sizeof(addr.sun_path)) {
        why = "socket path too long";
        return false;
    }
    std::error_code ec;
    std::filesystem::remove(path, ec);  // a stale socket from a crashed run
    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    addr.sun_family = AF_UNIX;
    std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
    if (listen_fd_ < 0 || ::bind(listen_fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0 || ::listen(listen_fd_, 4) != 0 ||
        ::pipe(wake_) != 0) {
        why = std::format("{}: {}", path, std::strerror(errno));
        if (listen_fd_ >= 0) ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    ::chmod(path.c_str(), 0600);
    socket_path_ = path;
    parent_pid_ = quit_with_parent ? ::getppid() : 0;
    acceptor_ = std::thread([this] { accept_loop(); });
    return true;
}

void Server::stop() {
    if (listen_fd_ < 0) return;
    stopping_ = true;
    [[maybe_unused]] const auto n = ::write(wake_[1], "x", 1);
    if (acceptor_.joinable()) acceptor_.join();
    {
        std::lock_guard lock(conn_mutex_);
        for (const int fd : conn_fds_) ::shutdown(fd, SHUT_RDWR);
    }
    for (auto& t : connections_)
        if (t.joinable()) t.join();
    ::close(listen_fd_);
    ::close(wake_[0]);
    ::close(wake_[1]);
    listen_fd_ = -1;
    std::error_code ec;
    std::filesystem::remove(socket_path_, ec);
}

void Server::accept_loop() {
    while (!stopping_) {
        pollfd fds[2] = {{listen_fd_, POLLIN, 0}, {wake_[0], POLLIN, 0}};
        if (::poll(fds, 2, -1) < 0 && errno != EINTR) return;
        if (stopping_ || (fds[1].revents & POLLIN)) return;
        if (!(fds[0].revents & POLLIN)) continue;
        const int fd = ::accept(listen_fd_, nullptr, nullptr);
        if (fd < 0) continue;
        std::lock_guard lock(conn_mutex_);
        conn_fds_.push_back(fd);
        connections_.emplace_back([this, fd] { serve(fd); });
    }
}

void Server::serve(int fd) {
    std::string buffer;
    char chunk[65536];
    for (;;) {
        const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
        if (n <= 0) break;
        buffer.append(chunk, static_cast<std::size_t>(n));
        for (std::size_t nl; (nl = buffer.find('\n')) != std::string::npos;) {
            std::string line = buffer.substr(0, nl);
            buffer.erase(0, nl + 1);
            if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
            // How long to wait for the UI thread: the call's own timeout, else the default.
            const json req = json::parse(line, nullptr, false);
            int timeout_ms = kDefaultTimeoutMs;
            json id;
            if (!req.is_discarded() && req.is_object()) {
                id = req.value("id", json());
                const json params = req.value("params", json::object());
                if (params.is_object()) {
                    timeout_ms = std::max(timeout_ms, params.value("timeout_ms", 0) + 2000);  // a wait's own timeout
                    if (params.contains("_timeout_ms")) timeout_ms = params.value("_timeout_ms", timeout_ms);  // the caller's say
                }
            }
            auto reply = submit(std::move(line));
            std::string out;
            if (reply.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready) {
                out = reply.get();
            } else {
                out = response_line(id, error(ErrorCode::busy, std::format("the {} app's UI thread did not answer within {} ms (a stuck frame?)", name_, timeout_ms)));
            }
            for (std::size_t sent = 0; sent < out.size();) {
                const ssize_t w = ::send(fd, out.data() + sent, out.size() - sent, 0);
                if (w <= 0) break;
                sent += static_cast<std::size_t>(w);
            }
        }
    }
    ::close(fd);
    std::lock_guard lock(conn_mutex_);
    std::erase(conn_fds_, fd);
}

}  // namespace einstar::agent
