#pragma once

// The agent-control endpoint embedded in an app (protocol.hpp). Off unless the app is started with
// --mcp=<socket>.
//
// The app owns the frame loop and calls pump() once per frame on its UI thread, before ImGui::NewFrame();
// every handler runs there and may touch the app's state directly. A socket thread only reads requests,
// queues them and waits: if the UI thread does not get to one in time the caller gets `busy` -- an answer
// about the app's health -- rather than a hang.
//
// A handler answers at once, or returns a Poll the server calls once per frame until it has an answer (a
// screenshot of the next frame, a wait, a click that takes several frames to play). A mutating method's
// answer is held for two more frames, so whatever the agent looks at next shows its effect.

#include <atomic>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "einstar/agent/image.hpp"
#include "einstar/agent/log_ring.hpp"
#include "einstar/agent/protocol.hpp"
#include "einstar/agent/ui.hpp"

namespace einstar::agent {

class Server {
public:
    using Poll = std::function<std::optional<Result>()>;
    using Outcome = std::variant<json, Error, Poll>;
    using Handler = std::function<Outcome(const json& params)>;

    // Thrown by a handler (or the param helpers below) for bad parameters: answered as invalid_params.
    struct BadParams : std::runtime_error {
        using std::runtime_error::runtime_error;
    };

    // `app` selects the methods this server may register (its own and the generic ones). Installs the log
    // sink and registers the generic methods (app.*, frame.*, ui.*, input.*).
    Server(App app, std::string name);
    ~Server();
    Server(const Server&) = delete;
    Server& operator=(const Server&) = delete;

    // Registers the handler of a method in method_specs() (std::logic_error otherwise: a method without a
    // spec would be invisible to the agent).
    void handle(std::string_view method, Handler handler);

    // Listens on `socket_path` (replacing a stale socket file; only this user may connect). False with the
    // reason in `why`. `quit_with_parent`: the app quits when the process that launched it goes away (an agent's
    // headless run); an app the user started for an agent to attach to passes false.
    bool start(const std::string& socket_path, std::string& why, bool quit_with_parent = true);
    void stop();

    // Once per frame on the UI thread, before ImGui::NewFrame().
    void pump();
    [[nodiscard]] std::uint64_t frames() const { return frames_; }

    // In process (tests): a request line in; its reply line once it is answered (keep pumping).
    std::future<std::string> submit(std::string line);

    // Screenshots: after drawing a frame, if capture_wanted(), the app hands over the frame's pixels.
    [[nodiscard]] bool capture_wanted() const { return capture_wanted_; }
    void provide_capture(Frame frame);

    [[nodiscard]] bool quit_requested() const { return quit_; }
    void set_visible(bool visible) { visible_ = visible; }

    [[nodiscard]] UiInspector& inspector() { return inspector_; }
    [[nodiscard]] InputPlayer& input() { return input_; }
    [[nodiscard]] LogRing& logs() { return logs_; }

    // Waits for the input up to `until` to be played, then answers `result` (handlers queueing input).
    Poll after_input(std::uint64_t until, json result = json::object());

private:
    struct Job;
    void start_job(const std::shared_ptr<Job>& job);
    void finish(const std::shared_ptr<Job>& job, Result result);
    void serve(int fd);
    void accept_loop();
    void register_builtins();
    [[nodiscard]] std::optional<ImVec2> point_of(const json& p);  // target / x, y -> window point

    App app_;
    std::string name_;
    std::map<std::string, Handler, std::less<>> handlers_;  // UI thread (registration before start())

    std::mutex mutex_;  // incoming_
    std::vector<std::shared_ptr<Job>> incoming_;
    std::vector<std::shared_ptr<Job>> active_, settling_;  // UI thread

    std::uint64_t frames_ = 0;
    UiInspector inspector_;
    InputPlayer input_;
    LogRing logs_;
    bool capture_wanted_ = false;
    std::optional<Frame> capture_;
    std::uint64_t capture_frame_ = 0;
    std::atomic<bool> quit_{false};
    bool visible_ = false;

    std::string socket_path_;
    int parent_pid_ = 0;  // at start(): quit if it goes away
    int listen_fd_ = -1;
    int wake_[2] = {-1, -1};  // self-pipe: stop() wakes the accept loop
    std::atomic<bool> stopping_{false};
    std::thread acceptor_;
    std::mutex conn_mutex_;
    std::vector<std::thread> connections_;
    std::vector<int> conn_fds_;
};

// Parameter helpers for handlers (BadParams on a wrong type or a missing required field).
template <typename T>
T param(const json& params, std::string_view key) {
    const auto it = params.find(key);
    if (it == params.end()) throw Server::BadParams(std::string("missing '") + std::string(key) + "'");
    try {
        return it->get<T>();
    } catch (const json::exception&) {
        throw Server::BadParams(std::string("'") + std::string(key) + "' has the wrong type");
    }
}
template <typename T>
std::optional<T> opt_param(const json& params, std::string_view key) {
    if (!params.contains(key) || params.at(key).is_null()) return std::nullopt;
    return param<T>(params, key);
}

}  // namespace einstar::agent
