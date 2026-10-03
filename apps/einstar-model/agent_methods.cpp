#include "agent_methods.hpp"

#include <array>
#include <cmath>
#include <cstring>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "einstar/model/photo_geometry.hpp"
#include "einstar/model/photo_render.hpp"

namespace einstar::modelapp {
namespace {

using agent::ErrorCode;
using agent::json;
using Outcome = agent::Server::Outcome;

Outcome answer(const model::Outcome& o) {
    if (o.ok) return o.result;
    return agent::error(o.refused ? ErrorCode::refused : ErrorCode::invalid_params, o.error);
}

// Answers once the background work started for the agent has finished.
Outcome wait_for(ModelApp& app) {
    return agent::Server::Poll([&app]() -> std::optional<agent::Result> {
        if (app.busy()) return std::nullopt;
        const auto o = app.take_outcome();
        if (!o) return agent::error(ErrorCode::internal, "the work finished without an outcome");
        if (o->ok) return agent::Result(o->result);
        return agent::Result(agent::error(o->refused ? ErrorCode::refused : ErrorCode::invalid_params, o->error));
    });
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
        if (spec.app != agent::App::model || spec.name == "model.view" || spec.name == "model.photo.get") continue;
        const std::string command = spec.name.substr(std::string("model.").size());
        if (command == "open") {
            // A scan may take minutes to process: started here, answered when it is done.
            server.handle(spec.name, [&app](const json& p) -> Outcome {
                if (!p.contains("path") || !p["path"].is_string()) throw agent::Server::BadParams("missing parameter 'path'");
                const std::filesystem::path path = p["path"].get<std::string>();
                if (path.extension() == ".emodel") return answer(app.run("open", p, model::Author::agent));
                if (app.busy()) return agent::error(ErrorCode::refused, "busy: " + app.busy_text());
                (void)app.take_outcome();
                app.open(path, p.value("fine", false));
                return wait_for(app);
            });
            continue;
        }
        server.handle(spec.name, [&app, command](const json& p) -> Outcome {
            if (app.busy()) return agent::error(ErrorCode::refused, "busy: " + app.busy_text() + " (try again when it is done)");
            if (!ModelApp::is_slow(command)) return answer(app.run(command, p, model::Author::agent));
            (void)app.take_outcome();
            app.start(command, p, model::Author::agent);
            return wait_for(app);
        });
    }

    // A photo as an image (decoded and drawn here; the document keeps the file's bytes).
    server.handle("model.photo.get", [&app](const json& p) -> Outcome {
        if (app.busy()) return agent::error(ErrorCode::refused, "busy: " + app.busy_text());
        if (!p.contains("photo")) throw agent::Server::BadParams("missing parameter 'photo'");
        const json& ref = p["photo"];
        const model::Photo* photo = nullptr;
        for (const auto& ph : app.doc.state().photos)
            if ((ref.is_number_integer() && ph.id == ref.get<int>()) || (ref.is_string() && ph.name == ref.get<std::string>())) photo = &ph;
        if (!photo) return agent::error(ErrorCode::not_found, "no photo " + ref.dump());
        model::PhotoRenderOptions o;
        o.max_size = p.value("max_size", 1600);
        o.annotations = p.value("annotations", true);
        o.grid = p.value("grid", false);
        if (p.value("overlay", false)) {
            if (!photo->camera) return agent::error(ErrorCode::refused, "'" + photo->name + "' is not registered (model.photo.correspond)");
            o.lines = model::overlay_lines(app.doc, *photo);
        }
        if (p.contains("crop")) {
            const auto c = p["crop"].get<std::vector<double>>();
            if (c.size() != 4) throw agent::Server::BadParams("crop is [x, y, w, h]");
            o.crop = std::array<double, 4>{c[0], c[1], c[2], c[3]};
        }
        const auto r = model::render_photo(*photo, o);
        if (!r) return agent::error(ErrorCode::internal, r.error().message);
        json annotations = json::array();
        for (const auto& a : photo->annotations) annotations.push_back(model::annotation_summary(a));
        return json{{"jpeg_base64", model::base64_encode(r->jpeg)},
                    {"photo", photo->name},
                    {"photo_size", {photo->width, photo->height}},
                    {"image_size", {r->width, r->height}},
                    {"scale", r->scale},
                    {"crop", r->crop},
                    {"annotations", annotations},
                    {"note", "an image pixel (px, py) is the photo pixel (crop.x + px / scale, crop.y + py / scale)"}};
    });

    server.handle("model.view", [&app, &camera](const json& p) -> Outcome {
        if (app.busy()) return agent::error(ErrorCode::refused, "busy: " + app.busy_text());
        if (p.contains("mode")) {
            const auto d = display_from_name(p["mode"].get<std::string>());
            if (!d) throw agent::Server::BadParams("mode is labels, deviation, model or photos");
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
