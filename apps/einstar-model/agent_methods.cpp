#include "agent_methods.hpp"

#include <cmath>
#include <cstring>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace einstar::modelapp {
namespace {

using agent::ErrorCode;
using agent::json;
using Outcome = agent::Server::Outcome;

Outcome answer(const model::Outcome& o) {
    if (o.ok) return o.result;
    return agent::error(o.refused ? ErrorCode::refused : ErrorCode::invalid_params, o.error);
}

json view_json(const ModelApp& app, const render::ViewCamera& c) {
    const auto v = [](const Vec3f& x) { return json{x.x(), x.y(), x.z()}; };
    return {{"mode", display_name(app.display)}, {"edges", app.show_edges}, {"target", v(c.target)}, {"distance", c.distance},
            {"eye", v(c.eye())}, {"forward", v(c.forward())}};
}

}  // namespace

bool agent_socket_in_use(const std::string& path) {
    sockaddr_un addr{};
    if (path.size() >= sizeof(addr.sun_path)) return false;
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return false;
    addr.sun_family = AF_UNIX;
    std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
    const bool live = ::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0;
    ::close(fd);
    return live;
}

void register_model_agent(agent::Server& server, ModelApp& app, render::ViewCamera& camera) {
    for (const auto& spec : agent::method_specs()) {
        if (spec.app != agent::App::model || spec.name == "model.view") continue;
        const std::string command = spec.name.substr(std::string("model.").size());
        if (command == "open") {
            // A scan may take minutes to process: started here, answered when it is done.
            server.handle(spec.name, [&app](const json& p) -> Outcome {
                if (!p.contains("path") || !p["path"].is_string()) throw agent::Server::BadParams("missing parameter 'path'");
                const std::filesystem::path path = p["path"].get<std::string>();
                if (path.extension() == ".emodel") return answer(app.run("open", p, model::Author::agent));
                if (app.busy()) return agent::error(ErrorCode::refused, "already opening a scan");
                (void)app.take_open_outcome();
                app.open(path, p.value("fine", false));
                return agent::Server::Poll([&app]() -> std::optional<agent::Result> {
                    if (app.busy()) return std::nullopt;
                    const auto o = app.take_open_outcome();
                    if (!o) return agent::error(ErrorCode::internal, "the open finished without an outcome");
                    if (o->ok) return agent::Result(o->result);
                    return agent::Result(agent::error(ErrorCode::refused, o->error));
                });
            });
            continue;
        }
        server.handle(spec.name, [&app, command](const json& p) -> Outcome {
            if (app.busy()) return agent::error(ErrorCode::refused, "a scan is being opened");
            return answer(app.run(command, p, model::Author::agent));
        });
    }

    server.handle("model.view", [&app, &camera](const json& p) -> Outcome {
        if (p.contains("mode")) {
            const auto d = display_from_name(p["mode"].get<std::string>());
            if (!d) throw agent::Server::BadParams("mode is labels, deviation or model");
            app.display = *d;
        }
        if (p.contains("edges")) app.show_edges = p["edges"].get<bool>();
        if (p.contains("preset") && !app.look_from(camera, p["preset"].get<std::string>())) throw agent::Server::BadParams("unknown preset");
        if (p.contains("frame")) {
            const auto f = p["frame"].get<std::string>();
            if (f != "all" && !app.doc.find_label(f)) return agent::error(ErrorCode::not_found, "no label '" + f + "'");
            app.frame(camera, f == "all" ? std::string() : f);
        }
        if (p.contains("orbit")) camera.orbit(p["orbit"][0].get<float>(), p["orbit"][1].get<float>());
        if (p.contains("zoom")) camera.zoom(std::clamp(p["zoom"].get<float>(), 0.01f, 100.0f));
        return view_json(app, camera);
    });
}

}  // namespace einstar::modelapp
