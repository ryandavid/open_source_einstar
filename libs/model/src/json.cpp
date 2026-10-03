#include "einstar/model/json.hpp"

#include <cmath>
#include <numbers>

namespace einstar::model {
namespace {

template <class... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};

double round_to(double v, double unit) { return std::round(v / unit) * unit; }

}  // namespace

nlohmann::json vec_to_json(const Vec3& v) {
    // Micrometres are plenty for a scan and keep the agent's view of the numbers readable.
    return {round_to(v.x(), 1e-4), round_to(v.y(), 1e-4), round_to(v.z(), 1e-4)};
}

Vec3 vec_from_json(const nlohmann::json& j) {
    if (!j.is_array() || j.size() != 3) throw nlohmann::json::type_error::create(302, "a point or vector is [x, y, z]", &j);
    return {j[0].get<double>(), j[1].get<double>(), j[2].get<double>()};
}

nlohmann::json surface_to_json(const fit::Surface& s) {
    // Directions keep full precision (they are unit vectors that constraints make exact).
    const auto dir = [](const Vec3& v) { return nlohmann::json{v.x(), v.y(), v.z()}; };
    return std::visit(Overloaded{
                          [&](const fit::Plane& p) { return nlohmann::json{{"kind", "plane"}, {"normal", dir(p.normal)}, {"offset", p.offset}}; },
                          [&](const fit::Cylinder& c) {
                              return nlohmann::json{{"kind", "cylinder"}, {"point", vec_to_json(c.point)}, {"axis", dir(c.axis)}, {"radius", c.radius},
                                                    {"diameter", 2 * c.radius}};
                          },
                          [&](const fit::Cone& c) {
                              return nlohmann::json{{"kind", "cone"}, {"apex", vec_to_json(c.apex)}, {"axis", dir(c.axis)},
                                                    {"half_angle_deg", c.half_angle * 180 / std::numbers::pi}};
                          },
                          [&](const fit::Sphere& sp) {
                              return nlohmann::json{{"kind", "sphere"}, {"center", vec_to_json(sp.center)}, {"radius", sp.radius}};
                          },
                          [&](const fit::Torus& t) {
                              return nlohmann::json{{"kind", "torus"}, {"center", vec_to_json(t.center)}, {"axis", dir(t.axis)}, {"major", t.major},
                                                    {"minor", t.minor}};
                          },
                          [&](const fit::Freeform& f) {
                              return nlohmann::json{{"kind", "freeform"}, {"frame", frame_to_json(f.frame)}, {"u0", f.u0}, {"v0", f.v0}, {"du", f.du},
                                                    {"dv", f.dv}, {"nu", f.nu}, {"nv", f.nv}, {"heights", *f.heights}};
                          },
                      },
                      s);
}

nlohmann::json surface_summary_json(const fit::Surface& s) {
    if (const auto* f = std::get_if<fit::Freeform>(&s))
        return {{"kind", "freeform"}, {"control_grid", {f->nu, f->nv}}, {"spacing_mm", f->du}, {"origin", vec_to_json(f->frame.translation())},
                {"normal", {f->frame.linear()(0, 2), f->frame.linear()(1, 2), f->frame.linear()(2, 2)}}};
    return surface_to_json(s);
}

fit::Surface surface_from_json(const nlohmann::json& j) {
    const auto kind = j.at("kind").get<std::string>();
    if (kind == "plane") return fit::Plane{vec_from_json(j.at("normal")).normalized(), j.at("offset").get<double>()};
    if (kind == "cylinder") return fit::Cylinder{vec_from_json(j.at("point")), vec_from_json(j.at("axis")).normalized(), j.at("radius").get<double>()};
    if (kind == "cone")
        return fit::Cone{vec_from_json(j.at("apex")), vec_from_json(j.at("axis")).normalized(), j.at("half_angle_deg").get<double>() * std::numbers::pi / 180};
    if (kind == "sphere") return fit::Sphere{vec_from_json(j.at("center")), j.at("radius").get<double>()};
    if (kind == "torus")
        return fit::Torus{vec_from_json(j.at("center")), vec_from_json(j.at("axis")).normalized(), j.at("major").get<double>(), j.at("minor").get<double>()};
    if (kind == "freeform") {
        fit::Freeform f;
        f.frame = frame_from_json(j.at("frame"));
        f.u0 = j.at("u0");
        f.v0 = j.at("v0");
        f.du = j.at("du");
        f.dv = j.at("dv");
        f.nu = j.at("nu");
        f.nv = j.at("nv");
        auto h = std::make_shared<std::vector<double>>(j.at("heights").get<std::vector<double>>());
        if (f.nu < 4 || f.nv < 4 || h->size() != static_cast<std::size_t>(f.nu * f.nv))
            throw nlohmann::json::type_error::create(302, "a freeform surface's heights do not match its grid", &j);
        f.heights = std::move(h);
        return f;
    }
    throw nlohmann::json::type_error::create(302, "unknown surface kind '" + kind + "'", &j);
}

std::vector<fit::SurfaceKind> kinds_from_json(const nlohmann::json& j) {
    std::vector<fit::SurfaceKind> out;
    for (const auto& k : j) {
        const auto name = k.get<std::string>();
        bool found = false;
        for (const auto kind : {fit::SurfaceKind::plane, fit::SurfaceKind::cylinder, fit::SurfaceKind::cone, fit::SurfaceKind::sphere, fit::SurfaceKind::torus,
                                fit::SurfaceKind::freeform})
            if (fit::kind_name(kind) == name) {
                out.push_back(kind);
                found = true;
            }
        if (!found) throw nlohmann::json::type_error::create(302, "unknown surface kind '" + name + "'", &k);
    }
    return out;
}

nlohmann::json frame_to_json(const SE3& T) {
    nlohmann::json m = nlohmann::json::array();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) m.push_back(T.matrix()(r, c));
    return m;
}

SE3 frame_from_json(const nlohmann::json& j) {
    SE3 T = SE3::Identity();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) T.matrix()(r, c) = j.at(static_cast<std::size_t>(r * 4 + c)).get<double>();
    return T;
}

}  // namespace einstar::model
