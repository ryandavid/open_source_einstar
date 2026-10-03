#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <map>
#include <numbers>
#include <set>

#include "einstar/fit/detect.hpp"
#include "einstar/fit/grow.hpp"
#include "einstar/fit/holes.hpp"
#include "einstar/fit/primitive_fit.hpp"
#include "einstar/fit/synthetic_part.hpp"
#include "einstar/model/document.hpp"
#include "einstar/model/io.hpp"
#include "einstar/model/json.hpp"

namespace einstar::model {

// A command that cannot be done: bad parameters (the default) or not now (refused).
struct CommandError {
    std::string message;
    bool refused = false;
};

struct Document::Impl {
    Document& d;

    // ---- references ----

    [[noreturn]] static void fail(std::string message) { throw CommandError{std::move(message), false}; }
    [[noreturn]] static void refuse(std::string message) { throw CommandError{std::move(message), true}; }

    static const json& need(const json& p, const char* key) {
        if (!p.is_object() || !p.contains(key)) fail(std::format("missing parameter '{}'", key));
        return p.at(key);
    }
    template <class T>
    static T get(const json& p, const char* key) {
        try {
            return need(p, key).get<T>();
        } catch (const json::exception&) {
            fail(std::format("parameter '{}' has the wrong type", key));
        }
    }
    template <class T>
    static std::optional<T> opt(const json& p, const char* key) {
        if (!p.is_object() || !p.contains(key) || p.at(key).is_null()) return std::nullopt;
        return get<T>(p, key);
    }

    void need_scan() const {
        if (!d.has_scan()) refuse("no scan is open (model.open or model.open_demo)");
    }

    Label& label_ref(const json& ref) {
        const Label* l = d.find_label(ref);
        if (!l) fail(std::format("no label {}", ref.dump()));
        return *std::ranges::find(d.state_.labels, l->id, &Label::id);
    }
    Datum& datum_ref(const json& ref) {
        for (auto& x : d.state_.datums)
            if ((ref.is_number_integer() && x.id == ref.get<int>()) || (ref.is_string() && x.name == ref.get<std::string>())) return x;
        fail(std::format("no datum {}", ref.dump()));
    }
    Hole& hole_ref(const json& ref) {
        for (auto& x : d.state_.holes)
            if ((ref.is_number_integer() && x.id == ref.get<int>()) || (ref.is_string() && x.name == ref.get<std::string>())) return x;
        fail(std::format("no hole {}", ref.dump()));
    }
    Fillet& fillet_ref(const json& ref) {
        for (auto& x : d.state_.fillets)
            if ((ref.is_number_integer() && x.id == ref.get<int>()) || (ref.is_string() && x.name == ref.get<std::string>())) return x;
        fail(std::format("no fillet {}", ref.dump()));
    }
    // A label's or a hole's name (constraints may refer to either).
    std::string label_name(int id) const {
        if (const Label* l = d.label(id)) return l->name;
        for (const auto& h : d.state_.holes)
            if (h.id == id) return h.name;
        return std::format("#{}", id);
    }
    // A label or, where axes are meant, a hole: its id.
    int entity_ref(const json& ref) {
        if (d.find_label(ref)) return d.find_label(ref)->id;
        for (const auto& h : d.state_.holes)
            if ((ref.is_number_integer() && h.id == ref.get<int>()) || (ref.is_string() && h.name == ref.get<std::string>())) return h.id;
        fail(std::format("no label or hole {}", ref.dump()));
    }
    std::string datum_name(int id) const {
        for (const auto& x : d.state_.datums)
            if (x.id == id) return x.name;
        return std::format("#{}", id);
    }

    bool name_taken(const std::string& name) const {
        return std::ranges::any_of(d.state_.labels, [&](const Label& l) { return l.name == name; }) ||
               std::ranges::any_of(d.state_.holes, [&](const Hole& h) { return h.name == name; }) ||
               std::ranges::any_of(d.state_.fillets, [&](const Fillet& f) { return f.name == name; }) ||
               std::ranges::any_of(d.state_.datums, [&](const Datum& x) { return x.name == name; });
    }
    std::string unique_name(const std::string& base) const {
        if (!name_taken(base)) return base;
        for (int i = 2;; ++i)
            if (const auto n = std::format("{} {}", base, i); !name_taken(n)) return n;
    }
    std::string checked_name(const json& p, const std::string& base) const {
        if (const auto n = opt<std::string>(p, "name")) {
            if (n->empty()) fail("a name cannot be empty");
            if (name_taken(*n)) fail(std::format("the name '{}' is taken", *n));
            return *n;
        }
        return unique_name(base);
    }
    int new_id() {
        const int id = d.state_.next_id++;
        if (id > 65535) refuse("too many objects in this document");
        return id;
    }

    // ---- per-triangle arrays ----

    std::vector<std::uint16_t> paint_copy() const { return *d.state_.paint; }
    std::vector<std::uint16_t> region_copy() const { return *d.state_.region; }
    void set_paint(std::vector<std::uint16_t> v) { d.state_.paint = std::make_shared<const std::vector<std::uint16_t>>(std::move(v)); }
    void set_region(std::vector<std::uint16_t> v) { d.state_.region = std::make_shared<const std::vector<std::uint16_t>>(std::move(v)); }

    std::vector<std::uint32_t> triangles_of(int id, bool painted = false) const {
        const auto& arr = painted ? *d.state_.paint : *d.state_.region;
        std::vector<std::uint32_t> out;
        for (std::uint32_t t = 0; t < arr.size(); ++t)
            if (arr[t] == id) out.push_back(t);
        return out;
    }

    // Shared edges between regions: (a, b) with a < b -> count.
    std::map<std::pair<int, int>, int> adjacency() const {
        std::map<std::pair<int, int>, int> adj;
        const auto& region = *d.state_.region;
        const auto& topo = d.topology();
        for (std::uint32_t t = 0; t < region.size(); ++t) {
            if (region[t] == 0) continue;
            for (const auto n : topo.neighbors(t))
                if (n != fit::kNoTriangle && n > t && region[n] != 0 && region[n] != region[t])
                    ++adj[{std::min<int>(region[t], region[n]), std::max<int>(region[t], region[n])}];
        }
        return adj;
    }

    // The surface a label stands for: the solved one, else its fit, else the given one.
    std::optional<fit::Surface> surface_of(const Label& l) const {
        if (const auto it = d.state_.solved.find(l.id); it != d.state_.solved.end()) return it->second;
        if (l.given) return l.given;
        return l.fit;
    }

    // ---- read-only ----

    json label_json(const Label& l, const std::vector<std::uint32_t>* counts, const std::vector<std::uint32_t>* painted) const {
        json j = {{"id", l.id}, {"name", l.name}, {"role", role_name(l.role)}};
        if (!l.kinds.empty()) {
            j["kinds"] = json::array();
            for (const auto k : l.kinds) j["kinds"].push_back(fit::kind_name(k));
        }
        if (counts) j["triangles"] = (*counts)[static_cast<std::size_t>(l.id)];
        if (painted) j["painted"] = (*painted)[static_cast<std::size_t>(l.id)];
        if (l.given) j["given"] = surface_to_json(*l.given);
        if (l.fit) {
            j["fit"] = surface_to_json(*l.fit);
            j["sigma_mm"] = l.sigma;
            j["rms_mm"] = l.rms;
        }
        if (const auto it = d.state_.solved.find(l.id); it != d.state_.solved.end()) j["solved"] = surface_to_json(it->second);
        return j;
    }

    json summary() const {
        json out = {{"revision", d.revision_}};
        if (!d.has_scan()) {
            out["scan"] = nullptr;
            return out;
        }
        Eigen::AlignedBox3f box;
        for (const auto& v : d.mesh().vertices) box.extend(v);
        out["scan"] = {{"kind", d.source_.kind},
                       {"path", d.source_.path.string()},
                       {"triangles", d.mesh().triangles.size()},
                       {"voxel_mm", d.voxel_mm()},
                       {"bounds", {{"min", vec_to_json(box.min().cast<double>())}, {"max", vec_to_json(box.max().cast<double>())}}}};
        std::vector<std::uint32_t> counts(65536, 0), painted(65536, 0);
        std::size_t unlabelled = 0;
        for (const auto r : *d.state_.region) {
            ++counts[r];
            unlabelled += r == 0;
        }
        for (const auto p : *d.state_.paint) ++painted[p];
        out["unlabelled_triangles"] = unlabelled;
        out["labels"] = json::array();
        for (const auto& l : d.state_.labels) out["labels"].push_back(label_json(l, &counts, &painted));
        out["holes"] = json::array();
        for (const auto& h : d.state_.holes) out["holes"].push_back(hole_json(h));
        out["fillets"] = json::array();
        for (const auto& f : d.state_.fillets) out["fillets"].push_back(fillet_json(f));
        out["datums"] = json::array();
        for (const auto& x : d.state_.datums)
            out["datums"].push_back({{"id", x.id}, {"name", x.name}, {"origin", vec_to_json(x.frame.translation())},
                                     {"x", vec_to_json(x.frame.linear().col(0))}, {"y", vec_to_json(x.frame.linear().col(1))},
                                     {"z", vec_to_json(x.frame.linear().col(2))}});
        out["constraints"] = json::array();
        for (const auto& c : d.state_.constraints) out["constraints"].push_back(constraint_json(c));
        if (d.built_) {
            const auto& r = d.built_->result;
            out["model"] = {{"current", d.built_is_current()}, {"ok", r.ok}, {"closed", r.closed}, {"volume_mm3", r.volume},
                            {"faces", r.faces}, {"edges", r.edges}, {"scan_coverage", r.scan_coverage}, {"log", r.log}};
        } else {
            out["model"] = nullptr;
        }
        out["undo_steps"] = d.undo_.size();
        out["redo_steps"] = d.redo_.size();
        return out;
    }

    json hole_json(const Hole& h) const {
        json j = {{"id", h.id},
                  {"name", h.name},
                  {"host", label_name(h.host)},
                  {"center", vec_to_json(h.center)},
                  {"axis", vec_to_json(h.axis)},
                  {"diameter", h.used_diameter()},
                  {"measured_diameter", h.measured_diameter},
                  {"measured_from", h.wall_seen ? "wall" : "opening"},
                  {"wall_seen_to_depth", h.seen_depth},
                  {"through", !h.depth.has_value()}};
        if (h.diameter) j["set_diameter"] = *h.diameter;
        if (h.depth) j["depth"] = *h.depth;
        if (h.wall_label) j["wall_label"] = label_name(h.wall_label);
        return j;
    }
    json fillet_json(const Fillet& f) const {
        json j = {{"id", f.id}, {"name", f.name}, {"between", {label_name(f.face_a), label_name(f.face_b)}}, {"radius", f.used_radius()},
                  {"measured_radius", f.measured_radius}};
        if (f.radius) j["set_radius"] = *f.radius;
        if (f.label) j["label"] = label_name(f.label);
        return j;
    }
    // A constraint with its label and datum ids shown as names, and its status from the last solve.
    json constraint_json(const ConstraintDef& c) const {
        json spec = c.spec;
        for (const char* key : {"label", "a", "b"})
            if (spec.contains(key) && spec[key].is_number_integer()) spec[key] = label_name(spec[key].get<int>());
        if (spec.contains("datum")) spec["datum"] = datum_name(spec["datum"].get<int>());
        if (spec.contains("axis") && spec["axis"].is_number_integer()) spec["axis"] = std::string(1, "xyz"[std::clamp(spec["axis"].get<int>(), 0, 2)]);
        if (spec.contains("hole")) {
            const int hole = spec["hole"].get<int>();
            for (const auto& h : d.state_.holes)
                if (h.id == hole) spec["hole"] = h.name;
        }
        json j = {{"id", c.id}, {"constraint", spec}};
        const std::string key = std::to_string(c.id);
        if (d.state_.solve_report.contains("constraints") && d.state_.solve_report["constraints"].contains(key))
            j["last_solve"] = d.state_.solve_report["constraints"][key];
        return j;
    }

    json label_info(const json& p) {
        need_scan();
        const Label& l = label_ref(need(p, "label"));
        json j = label_json(l, nullptr, nullptr);
        const auto tris = triangles_of(l.id);
        j["triangles"] = tris.size();
        j["painted"] = triangles_of(l.id, true).size();
        double area = 0;
        Eigen::AlignedBox3f box;
        Vec3 c = Vec3::Zero();
        for (const auto t : tris) {
            area += d.topology().area(t);
            c += d.topology().area(t) * d.topology().centroid(t).cast<double>();
            box.extend(d.topology().centroid(t));
        }
        j["area_mm2"] = area;
        if (area > 0) {
            j["centroid"] = vec_to_json(c / area);
            j["bounds"] = {{"min", vec_to_json(box.min().cast<double>())}, {"max", vec_to_json(box.max().cast<double>())}};
        }
        j["neighbours"] = json::array();
        for (const auto& [pair, n] : adjacency())
            if (pair.first == l.id || pair.second == l.id)
                j["neighbours"].push_back({{"label", label_name(pair.first == l.id ? pair.second : pair.first)}, {"shared_edges", n}});
        return j;
    }

    json deviation(const json& p) const {
        if (!d.built_) refuse("no model has been built yet (model.build)");
        const Built& b = *d.built_;
        const auto max_spots = opt<int>(p, "max_hot_spots").value_or(10);
        const bool edges = opt<bool>(p, "include_edge_bands").value_or(false);
        const auto stats = [&](const fit::DeviationStats& s) {
            return json{{"vertices", s.vertices}, {"rms_mm", s.rms}, {"p95_mm", s.p95}, {"max_mm", s.max_abs}, {"mean_mm", s.mean},
                        {"within_tolerance", s.within_tolerance}};
        };
        json out = {{"current", d.built_is_current()}, {"overall", stats(b.report.overall)}, {"unmatched_vertices", b.report.unmatched}};
        out["labels"] = json::array();
        for (const auto& g : b.report.groups) {
            json s = stats(g);
            s["label"] = label_name(g.group);
            out["labels"].push_back(s);
        }
        out["hot_spots"] = json::array();
        int bands = 0, shown = 0;
        for (const auto& h : b.report.hot_spots) {
            if (h.edge_band) ++bands;
            if ((h.edge_band && !edges) || shown >= max_spots) continue;
            ++shown;
            const auto face = h.model_face >= 0 && static_cast<std::size_t>(h.model_face) < b.result.face_names.size()
                                  ? b.result.face_names[static_cast<std::size_t>(h.model_face)]
                                  : std::string();
            out["hot_spots"].push_back({{"centroid", vec_to_json(h.centroid)},
                                        {"area_mm2", h.area_mm2},
                                        {"length_mm", h.length_mm},
                                        {"width_mm", h.width_mm},
                                        {"mean_mm", h.mean_mm},
                                        {"peak_mm", h.peak_mm},
                                        {"model_face", face},
                                        {"label", h.group > 0 ? label_name(h.group) : std::string()},
                                        {"edge_band", h.edge_band}});
        }
        out["edge_bands"] = bands;
        out["unsupported_model_faces"] = json::array();
        for (std::size_t f = 0; f < b.coverage.face_area.size(); ++f)
            if (b.coverage.face_area[f] > 0 && b.coverage.unsupported_area[f] > 0.2 * b.coverage.face_area[f])
                out["unsupported_model_faces"].push_back({{"face", b.result.face_names[f]},
                                                          {"unsupported_share", b.coverage.unsupported_area[f] / b.coverage.face_area[f]},
                                                          {"area_mm2", b.coverage.face_area[f]}});
        return out;
    }

    json raycast(const json& p) const {
        need_scan();
        const Vec3 o = vec_from_json(need(p, "origin")), dir = vec_from_json(need(p, "direction"));
        if (dir.norm() < 1e-12) fail("the direction is zero");
        const auto hit = d.bvh().raycast(o.cast<float>(), dir.normalized().cast<float>());
        if (!hit) return {{"hit", false}};
        const int l = (*d.state_.region)[hit->triangle];
        return {{"hit", true}, {"point", vec_to_json(hit->point.cast<double>())}, {"triangle", hit->triangle},
                {"label", l ? json(label_name(l)) : json(nullptr)}};
    }

    // ---- labels ----

    json label_create(const json& p) {
        need_scan();
        Label l;
        l.id = new_id();
        l.name = checked_name(p, "label");
        if (const auto r = opt<std::string>(p, "role")) {
            const auto role = role_from_name(*r);
            if (!role) fail(std::format("unknown role '{}' (face, hole, fillet, ignore)", *r));
            l.role = *role;
        }
        if (p.contains("kinds")) l.kinds = kinds_from_json(p["kinds"]);
        d.state_.labels.push_back(l);
        return {{"id", l.id}, {"name", l.name}};
    }

    json label_update(const json& p) {
        Label& l = label_ref(need(p, "label"));
        if (const auto n = opt<std::string>(p, "name"); n && *n != l.name) {
            if (n->empty() || name_taken(*n)) fail(std::format("the name '{}' is taken or empty", *n));
            l.name = *n;
        }
        if (const auto r = opt<std::string>(p, "role")) {
            const auto role = role_from_name(*r);
            if (!role) fail(std::format("unknown role '{}' (face, hole, fillet, ignore)", *r));
            l.role = *role;
        }
        if (p.contains("kinds")) l.kinds = kinds_from_json(p["kinds"]);
        return label_json(l, nullptr, nullptr);
    }

    json label_delete(const json& p) {
        const int id = label_ref(need(p, "label")).id;
        auto paint = paint_copy();
        auto region = region_copy();
        for (auto& x : paint) x = x == id ? 0 : x;
        for (auto& x : region) x = x == id ? 0 : x;
        set_paint(std::move(paint));
        set_region(std::move(region));
        std::erase_if(d.state_.labels, [&](const Label& l) { return l.id == id; });
        std::vector<int> holes_gone;
        for (const auto& h : d.state_.holes)
            if (h.host == id) holes_gone.push_back(h.id);
        std::erase_if(d.state_.holes, [&](const Hole& h) { return h.host == id; });
        for (const int h : holes_gone)
            std::erase_if(d.state_.constraints, [&](const ConstraintDef& c) {
                return refers_to_label(c.spec, h) || (c.spec.contains("hole") && c.spec["hole"] == h);
            });
        for (auto& h : d.state_.holes)
            if (h.wall_label == id) h.wall_label = 0;
        std::erase_if(d.state_.fillets, [&](const Fillet& f) { return f.face_a == id || f.face_b == id; });
        for (auto& f : d.state_.fillets)
            if (f.label == id) f.label = 0;
        const auto removed = std::erase_if(d.state_.constraints, [&](const ConstraintDef& c) { return refers_to_label(c.spec, id); });
        d.state_.solved.erase(id);
        return {{"deleted", id}, {"constraints_removed", removed}};
    }

    static bool refers_to_label(const json& spec, int id) {
        for (const char* key : {"label", "a", "b"})
            if (spec.contains(key) && spec[key].is_number_integer() && spec[key].get<int>() == id) return true;
        return false;
    }

    json paint(const json& p) {
        need_scan();
        const bool erase = opt<bool>(p, "erase").value_or(false);
        int id = 0;
        if (p.contains("label")) id = label_ref(p["label"]).id;
        else if (!erase) fail("missing parameter 'label'");
        std::vector<std::uint32_t> tris;
        if (p.contains("triangles")) {
            for (const auto& t : p["triangles"]) {
                const auto i = t.get<std::uint32_t>();
                if (i >= d.mesh().triangles.size()) fail(std::format("no triangle {}", i));
                tris.push_back(i);
            }
        } else {
            const Vec3f point = vec_from_json(need(p, "point")).cast<float>();
            const auto radius = static_cast<float>(get<double>(p, "radius"));
            if (!(radius > 0)) fail("the radius must be positive");
            const auto hit = d.bvh().closest(point, radius + 5.0f);
            if (!hit) refuse("no scan within 5 mm of the point");
            tris = fit::triangles_within(d.topology(), hit->triangle, hit->point, radius);
        }
        auto paint = paint_copy();
        std::size_t changed = 0;
        for (const auto t : tris) {
            if (erase) {
                if (paint[t] != 0 && (id == 0 || paint[t] == id)) {
                    paint[t] = 0;
                    ++changed;
                }
            } else if (paint[t] != id) {
                paint[t] = static_cast<std::uint16_t>(id);
                ++changed;
            }
        }
        set_paint(std::move(paint));
        return {{"triangles", changed}};
    }

    // Grows every label that has paint (all at once, so neighbours share their boundary), refitting each.
    json grow(const json& p) {
        need_scan();
        std::set<int> only;
        if (p.contains("labels"))
            for (const auto& ref : p["labels"]) only.insert(label_ref(ref).id);
        std::vector<int> ids;
        std::vector<fit::RegionSeed> seeds;
        auto region = region_copy();
        std::vector<std::uint8_t> blocked(region.size(), 0);
        for (const auto& l : d.state_.labels) {
            if (!only.empty() && !only.contains(l.id)) continue;
            if (l.role == Role::ignore || l.given) continue;
            auto painted = triangles_of(l.id, true);
            if (painted.empty()) continue;
            ids.push_back(l.id);
            seeds.push_back({std::move(painted), l.kinds, std::nullopt});
        }
        if (seeds.empty()) refuse("no label to grow has paint (model.paint)");
        // Labels not grown now keep their regions, and their triangles are not taken.
        const std::set<int> growing(ids.begin(), ids.end());
        for (std::size_t t = 0; t < region.size(); ++t) {
            if (growing.contains(region[t])) region[t] = 0;
            const int pt = (*d.state_.paint)[t];
            if ((region[t] != 0 && !growing.contains(region[t])) || (pt != 0 && !growing.contains(pt))) blocked[t] = 1;
        }
        fit::GrowOptions go;
        const fit::GrowResult g = fit::grow_regions(d.topology(), seeds, go, blocked);
        json out = json::array();
        for (std::size_t i = 0; i < ids.size(); ++i) {
            Label& l = *std::ranges::find(d.state_.labels, ids[i], &Label::id);
            for (const auto t : g.regions[i].triangles) region[t] = static_cast<std::uint16_t>(l.id);
            if (g.regions[i].ok) {
                l.fit = g.regions[i].fit.surface;
                l.sigma = g.regions[i].fit.sigma;
                l.rms = g.regions[i].fit.rms;
                d.state_.solved.erase(l.id);
            }
            json j = {{"label", l.name}, {"triangles", g.regions[i].triangles.size()}, {"ok", g.regions[i].ok}};
            if (g.regions[i].ok) j["fit"] = surface_to_json(*l.fit);
            out.push_back(j);
        }
        set_region(std::move(region));
        return {{"grown", out}};
    }

    // Proposes labels for the unlabelled scan, sorts out fillets and hole walls, and finds the holes.
    json detect(const json& /*p*/) {
        need_scan();
        auto region = region_copy();
        auto paint = paint_copy();
        std::vector<std::uint8_t> blocked(region.size(), 0);
        for (std::size_t t = 0; t < region.size(); ++t) blocked[t] = region[t] != 0 || paint[t] != 0;
        fit::DetectOptions o;
        o.blocked = blocked;
        // At the scan's resolution: tighter curves than two voxels are rounded edges, smaller curved patches
        // than about 32 voxels are rims and corners (left for the user to brush if they matter).
        // Faces are proposed if they are also a visible share of the part (0.2% of its area for planes, 0.1%
        // for curved ones): an organic part, which no set of primitives describes, does not turn into thousands
        // of small patches.
        const double voxel = d.voxel_mm();
        double total_area = 0;
        for (const float a : d.topology().areas()) total_area += a;
        o.min_radius_mm = 2 * voxel;
        o.min_curved_area_mm2 = std::max({o.min_curved_area_mm2, 32 * voxel * voxel, 0.001 * total_area});
        o.min_plane_area_mm2 = std::max({o.min_plane_area_mm2, 32 * voxel * voxel, 0.002 * total_area});
        const auto found = fit::detect_regions(d.topology(), o);
        std::map<fit::SurfaceKind, int> counter;
        std::vector<int> created;
        for (const auto& r : found) {
            const auto kind = fit::kind_of(r.fit.surface);
            Label l;
            l.id = new_id();
            l.name = unique_name(std::format("{} {}", fit::kind_name(kind), ++counter[kind]));
            l.fit = r.fit.surface;
            l.sigma = r.fit.sigma;
            l.rms = r.fit.rms;
            for (const auto t : r.triangles) region[t] = paint[t] = static_cast<std::uint16_t>(l.id);
            created.push_back(l.id);
            d.state_.labels.push_back(std::move(l));
        }
        set_region(std::move(region));
        set_paint(std::move(paint));
        const json roles = classify(created);
        json holes = json::array();
        for (const int id : created) {
            const Label* l = d.label(id);
            if (l && l->role == Role::face && l->fit && fit::kind_of(*l->fit) == fit::SurfaceKind::plane)
                for (auto& h : find_holes_on(id)) holes.push_back(h);
        }
        json labels = json::array();
        for (const int id : created) labels.push_back(label_json(*d.label(id), nullptr, nullptr));
        return {{"labels", labels}, {"roles", roles}, {"holes", holes}};
    }

    // Material side of a curved label: +1 convex (material inside the surface), -1 concave (a hole wall).
    int convexity(const Label& l) const {
        if (!l.fit) return 0;
        double s = 0;
        const auto& mesh = d.mesh();
        for (const auto t : triangles_of(l.id)) {
            const auto& tri = mesh.triangles[t];
            const Vec3 c = d.topology().centroid(t).cast<double>();
            s += fit::normal_at(*l.fit, c).dot((mesh.normals[tri[0]] + mesh.normals[tri[1]] + mesh.normals[tri[2]]).cast<double>());
        }
        return s > 0 ? 1 : s < 0 ? -1 : 0;
    }

    // Among the given labels: cylinders running along the edge between two planes become fillets, concave
    // cylinders along a plane's normal become hole walls, small spheres and tori next to fillets become the
    // fillets' corner blends.
    json classify(const std::vector<int>& ids) {
        json out = json::array();
        const auto adj = adjacency();
        const auto neighbours = [&](int id) {
            std::vector<std::pair<int, int>> n;  // (label, shared edges)
            for (const auto& [pair, c] : adj)
                if (pair.first == id || pair.second == id) n.emplace_back(pair.first == id ? pair.second : pair.first, c);
            std::ranges::sort(n, [](const auto& a, const auto& b) { return a.second > b.second; });
            return n;
        };
        const auto is_plane = [&](int id) {
            const Label* l = d.label(id);
            return l && l->role == Role::face && l->fit && fit::kind_of(*l->fit) == fit::SurfaceKind::plane;
        };
        for (const int id : ids) {
            Label& l = *std::ranges::find(d.state_.labels, id, &Label::id);
            const auto* cyl = l.fit ? std::get_if<fit::Cylinder>(&*l.fit) : nullptr;
            if (!cyl) continue;
            const auto nb = neighbours(id);
            const int side = convexity(l);
            // A hole wall: concave, along the normal of a plane it opens into.
            if (side < 0) {
                for (const auto& [other, n] : nb)
                    if (is_plane(other) && std::abs(std::get<fit::Plane>(*d.label(other)->fit).normal.dot(cyl->axis)) > 0.99) {
                        l.role = Role::hole;
                        out.push_back({{"label", l.name}, {"role", "hole"}});
                        break;
                    }
                if (l.role == Role::hole) continue;
            }
            // A fillet: between two planes both parallel to its axis.
            std::vector<int> planes;
            for (const auto& [other, n] : nb)
                if (is_plane(other) && std::abs(std::get<fit::Plane>(*d.label(other)->fit).normal.dot(cyl->axis)) < 0.05) planes.push_back(other);
            if (planes.size() >= 2) {
                l.role = Role::fillet;
                Fillet f;
                f.id = new_id();
                f.name = unique_name(std::format("fillet {} {}", d.label(planes[0])->name, d.label(planes[1])->name));
                f.label = id;
                f.face_a = planes[0];
                f.face_b = planes[1];
                f.measured_radius = cyl->radius;
                d.state_.fillets.push_back(f);
                out.push_back({{"label", l.name}, {"role", "fillet"}, {"between", {d.label(planes[0])->name, d.label(planes[1])->name}}});
            }
        }
        // Corner blends: spheres and tori touching a fillet with about its radius.
        for (const int id : ids) {
            Label& l = *std::ranges::find(d.state_.labels, id, &Label::id);
            if (!l.fit || l.role != Role::face) continue;
            double r = 0;
            if (const auto* s = std::get_if<fit::Sphere>(&*l.fit)) r = s->radius;
            else if (const auto* t = std::get_if<fit::Torus>(&*l.fit)) r = t->minor;
            else continue;
            for (const auto& [other, n] : neighbours(id)) {
                const auto f = std::ranges::find(d.state_.fillets, other, &Fillet::label);
                if (f != d.state_.fillets.end() && std::abs(r - f->measured_radius) < 0.25 * f->measured_radius) {
                    l.role = Role::fillet;
                    out.push_back({{"label", l.name}, {"role", "fillet"}, {"corner_of", f->name}});
                    break;
                }
            }
        }
        return out;
    }

    std::vector<json> find_holes_on(int host_id) {
        const Label* host = d.label(host_id);
        const auto plane = surface_of(*host);
        if (!plane || fit::kind_of(*plane) != fit::SurfaceKind::plane) fail(std::format("'{}' is not a plane", host->name));
        fit::HoleOptions ho;
        ho.voxel_mm = d.voxel_mm();
        const auto found = fit::find_holes(d.mesh(), d.topology(), d.bvh(), triangles_of(host_id), std::get<fit::Plane>(*plane), ho);
        std::vector<json> out;
        for (const auto& c : found) {
            // Already known (found again)?
            if (std::ranges::any_of(d.state_.holes, [&](const Hole& h) { return (h.center - c.center).norm() < 0.5 * c.diameter && h.host == host_id; }))
                continue;
            Hole h;
            h.id = new_id();
            h.name = unique_name(std::format("hole {}", d.state_.holes.size() + 1));
            h.host = host_id;
            h.center = c.center;
            h.axis = c.axis;
            h.measured_diameter = c.diameter;
            h.wall_seen = c.wall_diameter.has_value();
            h.rim = c.opening;
            h.seen_depth = c.wall_depth;
            if (c.floor_depth) h.depth = *c.floor_depth;
            // The label most of its wall belongs to.
            std::map<int, int> votes;
            for (const auto t : c.wall)
                if (const int r = (*d.state_.region)[t]; r != 0 && r != host_id) ++votes[r];
            if (!votes.empty()) {
                const auto best = std::ranges::max_element(votes, {}, &std::pair<const int, int>::second);
                if (best->second * 2 > static_cast<int>(c.wall.size())) {
                    h.wall_label = best->first;
                    Label& wl = *std::ranges::find(d.state_.labels, best->first, &Label::id);
                    wl.role = Role::hole;
                }
            }
            d.state_.holes.push_back(h);
            out.push_back(hole_json(h));
        }
        return out;
    }

    json find_holes(const json& p) {
        need_scan();
        const int id = label_ref(need(p, "label")).id;
        json out = json::array();
        for (auto& h : find_holes_on(id)) out.push_back(h);
        return {{"holes", out}};
    }

    json hole_update(const json& p) {
        Hole& h = hole_ref(need(p, "hole"));
        if (p.contains("name")) {
            const auto n = get<std::string>(p, "name");
            if (n != h.name && (n.empty() || name_taken(n))) fail(std::format("the name '{}' is taken or empty", n));
            h.name = n;
        }
        if (p.contains("diameter")) {
            if (p["diameter"].is_null()) h.diameter.reset();
            else {
                const auto v = get<double>(p, "diameter");
                if (!(v > 0)) fail("the diameter must be positive");
                h.diameter = v;
            }
        }
        if (opt<bool>(p, "through").value_or(false)) h.depth.reset();
        if (const auto depth = opt<double>(p, "depth")) {
            if (!(*depth > 0)) fail("the depth must be positive");
            h.depth = *depth;
        }
        return hole_json(h);
    }

    json hole_delete(const json& p) {
        const int id = hole_ref(need(p, "hole")).id;
        std::erase_if(d.state_.holes, [&](const Hole& h) { return h.id == id; });
        const auto removed = std::erase_if(d.state_.constraints, [&](const ConstraintDef& c) {
            return refers_to_label(c.spec, id) || (c.spec.contains("hole") && c.spec["hole"] == id);
        });
        return {{"deleted", id}, {"constraints_removed", removed}};
    }

    json fillet_add(const json& p) {
        need_scan();
        Fillet f;
        if (p.contains("label")) {
            Label& l = label_ref(p["label"]);
            f.label = l.id;
            l.role = Role::fillet;
            if (l.fit) {
                if (const auto* c = std::get_if<fit::Cylinder>(&*l.fit)) f.measured_radius = c->radius;
                else if (const auto* t = std::get_if<fit::Torus>(&*l.fit)) f.measured_radius = t->minor;
            }
        }
        if (p.contains("a") && p.contains("b")) {
            f.face_a = label_ref(p["a"]).id;
            f.face_b = label_ref(p["b"]).id;
        } else if (f.label) {
            // The two face labels it shares most edges with.
            std::vector<std::pair<int, int>> nb;
            for (const auto& [pair, n] : adjacency())
                if (pair.first == f.label || pair.second == f.label) {
                    const int other = pair.first == f.label ? pair.second : pair.first;
                    if (const Label* o = d.label(other); o && o->role == Role::face) nb.emplace_back(other, n);
                }
            std::ranges::sort(nb, [](const auto& x, const auto& y) { return x.second > y.second; });
            if (nb.size() < 2) refuse("the fillet's label does not touch two face labels; give 'a' and 'b'");
            f.face_a = nb[0].first;
            f.face_b = nb[1].first;
        } else {
            fail("give the fillet's 'label', or the two faces 'a' and 'b'");
        }
        if (const auto r = opt<double>(p, "radius")) {
            if (!(*r > 0)) fail("the radius must be positive");
            f.radius = *r;
        }
        if (f.used_radius() <= 0) fail("no radius: give 'radius' (the label has no cylinder fit to measure it)");
        f.id = new_id();
        f.name = checked_name(p, std::format("fillet {} {}", label_name(f.face_a), label_name(f.face_b)));
        d.state_.fillets.push_back(f);
        return fillet_json(f);
    }

    json fillet_update(const json& p) {
        Fillet& f = fillet_ref(need(p, "fillet"));
        if (p.contains("radius")) {
            if (p["radius"].is_null()) f.radius.reset();
            else {
                const auto r = get<double>(p, "radius");
                if (!(r > 0)) fail("the radius must be positive");
                f.radius = r;
            }
        }
        return fillet_json(f);
    }

    json fillet_delete(const json& p) {
        const int id = fillet_ref(need(p, "fillet")).id;
        std::erase_if(d.state_.fillets, [&](const Fillet& f) { return f.id == id; });
        return {{"deleted", id}};
    }

    // ---- datums, given faces, constraints ----

    json datum_create(const json& p) {
        need_scan();
        const Label& z = label_ref(need(p, "z"));
        const Label& x = label_ref(need(p, "x"));
        const auto sz = surface_of(z), sx = surface_of(x);
        if (!sz || !sx) refuse("both labels need a fitted surface (grow them first)");
        if (!fit::direction_of(*sz) || !fit::direction_of(*sx)) fail("both labels need a direction (a plane or an axis)");
        Datum datum;
        datum.id = new_id();
        datum.name = checked_name(p, "datum");
        datum.frame = fit::datum_from(*sz, 2, *sx);
        d.state_.datums.push_back(datum);
        return {{"id", datum.id}, {"name", datum.name}};
    }

    json datum_delete(const json& p) {
        const int id = datum_ref(need(p, "datum")).id;
        std::erase_if(d.state_.datums, [&](const Datum& x) { return x.id == id; });
        const auto removed = std::erase_if(d.state_.constraints, [&](const ConstraintDef& c) {
            return c.spec.contains("datum") && c.spec["datum"].get<int>() == id;
        });
        return {{"deleted", id}, {"constraints_removed", removed}};
    }

    static int axis_index(const json& v) {
        if (v.is_number_integer()) {
            const int a = v.get<int>();
            if (a >= 0 && a <= 2) return a;
        } else if (v.is_string()) {
            const auto s = v.get<std::string>();
            if (s == "x") return 0;
            if (s == "y") return 1;
            if (s == "z") return 2;
        }
        fail("an axis is 'x', 'y' or 'z'");
    }

    json face_add_plane(const json& p) {
        need_scan();
        const Datum& datum = datum_ref(need(p, "datum"));
        const int axis = axis_index(need(p, "axis"));
        const double offset = get<double>(p, "offset");
        const std::string facing = opt<std::string>(p, "facing").value_or("+");
        if (facing != "+" && facing != "-") fail("facing is '+' or '-' (along or against the datum axis)");
        const Vec3 e = datum.frame.linear().col(axis);
        const Vec3 n = facing == "+" ? e : Vec3(-e);
        const Vec3 point = datum.frame.translation() + offset * e;
        Label l;
        l.id = new_id();
        l.name = checked_name(p, "face");
        l.given = fit::Plane{n, n.dot(point)};
        d.state_.labels.push_back(l);
        // It moves with the datum: aligned and offset.
        d.state_.constraints.push_back({new_id(), {{"type", "aligned"}, {"label", l.id}, {"datum", datum.id}, {"axis", axis}}});
        d.state_.constraints.push_back({new_id(), {{"type", "offset"}, {"label", l.id}, {"datum", datum.id}, {"axis", axis}, {"value", offset}}});
        return {{"id", l.id}, {"name", l.name}};
    }

    // Normalises a constraint given with names into ids; checks what it refers to exists.
    json constraint_spec(const json& p) {
        const auto type = get<std::string>(p, "type");
        json spec = {{"type", type}};
        const auto lbl = [&](const char* key) { spec[key] = label_ref(need(p, key)).id; };
        const auto any = [&](const char* key) { spec[key] = entity_ref(need(p, key)); };  // a label or a hole
        const auto value = [&](bool positive) {
            const auto v = get<double>(p, "value");
            if (positive && !(v > 0)) fail("the value must be positive");
            spec["value"] = v;
        };
        if (type == "aligned") {
            any("label");
            spec["datum"] = datum_ref(need(p, "datum")).id;
            spec["axis"] = axis_index(need(p, "axis"));
        } else if (type == "coplanar") {
            lbl("a");
            lbl("b");
        } else if (type == "parallel" || type == "perpendicular" || type == "coaxial") {
            any("a");
            any("b");
        } else if (type == "axis_distance") {
            any("a");
            any("b");
            value(true);
        } else if (type == "angle") {
            any("a");
            any("b");
            spec["degrees"] = get<double>(p, "degrees");
        } else if (type == "radius") {
            lbl("label");
            value(true);
        } else if (type == "diameter") {
            if (p.contains("hole")) spec["hole"] = hole_ref(p["hole"]).id;
            else lbl("label");
            value(true);
        } else if (type == "distance") {
            lbl("a");
            lbl("b");
            value(true);
        } else if (type == "offset") {
            any("label");
            spec["datum"] = datum_ref(need(p, "datum")).id;
            spec["axis"] = axis_index(need(p, "axis"));
            value(false);
        } else {
            fail(std::format("unknown constraint type '{}'", type));
        }
        return spec;
    }

    json constraint_add(const json& p) {
        need_scan();
        json spec = constraint_spec(p);
        // A measured hole diameter is the hole's dimension: it sets the hole (and its wall, if labelled).
        if (spec["type"] == "diameter" && spec.contains("hole")) {
            Hole& h = hole_ref(spec["hole"]);
            h.diameter = spec["value"].get<double>();
        }
        const int id = new_id();
        d.state_.constraints.push_back({id, spec});
        return constraint_json(d.state_.constraints.back());
    }

    json constraint_remove(const json& p) {
        const int id = get<int>(p, "constraint");
        const auto it = std::ranges::find(d.state_.constraints, id, &ConstraintDef::id);
        if (it == d.state_.constraints.end()) fail(std::format("no constraint {}", id));
        if (it->spec["type"] == "diameter" && it->spec.contains("hole"))
            for (auto& h : d.state_.holes)
                if (h.id == it->spec["hole"].get<int>()) h.diameter.reset();
        d.state_.constraints.erase(it);
        return {{"removed", id}};
    }

    // Aligns plane labels to the datum axis nearest their normal ("make the box square").
    json square(const json& p) {
        need_scan();
        const Datum& datum = datum_ref(need(p, "datum"));
        const double max_angle = opt<double>(p, "max_angle_deg").value_or(10.0);
        std::vector<int> ids;
        if (p.contains("labels")) {
            for (const auto& ref : p["labels"]) ids.push_back(label_ref(ref).id);
        } else {
            for (const auto& l : d.state_.labels)
                if (l.role == Role::face && !l.given && l.fit && fit::kind_of(*l.fit) == fit::SurfaceKind::plane) ids.push_back(l.id);
        }
        json added = json::array(), skipped = json::array();
        for (const int id : ids) {
            const Label& l = *d.label(id);
            const auto s = surface_of(l);
            const auto dir = s ? fit::direction_of(*s) : std::nullopt;
            if (!dir) {
                skipped.push_back({{"label", l.name}, {"why", "no direction"}});
                continue;
            }
            int best = 0;
            double best_dot = 0;
            for (int k = 0; k < 3; ++k)
                if (const double c = std::abs(dir->dot(datum.frame.linear().col(k))); c > best_dot) {
                    best_dot = c;
                    best = k;
                }
            const double angle = std::acos(std::min(1.0, best_dot)) * 180.0 / std::numbers::pi;
            if (angle > max_angle) {
                skipped.push_back({{"label", l.name}, {"why", std::format("{:.1f} deg from the nearest axis", angle)}});
                continue;
            }
            const bool exists = std::ranges::any_of(d.state_.constraints, [&](const ConstraintDef& c) {
                return c.spec["type"] == "aligned" && c.spec["label"] == id && c.spec["datum"] == datum.id;
            });
            if (exists) continue;
            d.state_.constraints.push_back({new_id(), {{"type", "aligned"}, {"label", id}, {"datum", datum.id}, {"axis", best}}});
            added.push_back({{"label", l.name}, {"axis", std::string(1, "xyz"[best])}, {"was_off_deg", angle}});
        }
        return {{"added", added}, {"skipped", skipped}};
    }

    // ---- solve and build ----

    json solve(const json& /*p*/) {
        need_scan();
        fit::SolveInput in;
        std::map<int, int> feature_of;  // label id -> feature index
        std::vector<int> label_of;
        for (const auto& l : d.state_.labels) {
            if (l.role == Role::ignore) continue;
            if (l.given) {
                feature_of[l.id] = static_cast<int>(in.features.size());
                label_of.push_back(l.id);
                in.features.push_back({*l.given, {}, {}, false});
                continue;
            }
            if (!l.fit) continue;
            const fit::RegionPoints pts = fit::region_points(d.topology(), triangles_of(l.id), 3000);
            if (pts.points.empty()) continue;
            feature_of[l.id] = static_cast<int>(in.features.size());
            label_of.push_back(l.id);
            in.features.push_back({*l.fit, pts.points, pts.weights, false});
        }
        // Holes take part through their axis: the wall's label when it was labelled, otherwise a cylinder fitted
        // to the opening (its position, not its size: the opening is wider than the hole). Each stays parallel to
        // its host face's normal.
        std::map<int, int> rim_feature;  // hole id -> feature fitted to its opening
        std::vector<std::pair<int, int>> hole_host;  // feature pairs held parallel
        for (const auto& h : d.state_.holes) {
            int f = -1;
            if (h.wall_label && feature_of.contains(h.wall_label)) {
                f = feature_of[h.wall_label];
            } else if (h.rim.size() >= 6) {
                std::vector<Vec3> pts;
                double r = 0;
                for (const auto v : h.rim) {
                    pts.push_back(d.mesh().vertices[v].cast<double>());
                    const Vec3 q = pts.back() - h.center;
                    r += (q - q.dot(h.axis) * h.axis).norm();
                }
                r /= static_cast<double>(pts.size());
                f = static_cast<int>(in.features.size());
                rim_feature[h.id] = f;
                in.features.push_back({fit::Cylinder{h.center, h.axis, r}, std::move(pts), {}, false});
            }
            if (f < 0) continue;
            feature_of[h.id] = f;
            if (feature_of.contains(h.host)) hole_host.emplace_back(f, feature_of[h.host]);
        }
        std::map<int, int> datum_index;
        for (const auto& x : d.state_.datums) {
            datum_index[x.id] = static_cast<int>(in.datums.size());
            in.datums.push_back({x.frame, false});
        }
        for (const auto& [hole, host] : hole_host) in.constraints.push_back(fit::Parallel{hole, host});
        const std::size_t implicit = in.constraints.size();
        std::vector<int> constraint_of;  // input index (after the implicit ones) -> constraint id
        json skipped = json::array();
        for (const auto& c : d.state_.constraints) {
            const auto& s = c.spec;
            const auto f = [&](const char* key) -> int {
                const auto it = feature_of.find(s[key].get<int>());
                return it == feature_of.end() ? -1 : it->second;
            };
            const auto dt = [&]() { return datum_index.at(s["datum"].get<int>()); };
            const std::string type = s["type"];
            std::optional<fit::Constraint> fc;
            if (type == "diameter" && s.contains("hole")) continue;  // applied to the hole at build
            if (type == "aligned") fc = fit::Aligned{f("label"), dt(), s["axis"].get<int>()};
            else if (type == "parallel") fc = fit::Parallel{f("a"), f("b")};
            else if (type == "perpendicular") fc = fit::Perpendicular{f("a"), f("b")};
            else if (type == "angle") fc = fit::Angle{f("a"), f("b"), s["degrees"].get<double>()};
            else if (type == "coplanar") fc = fit::Coplanar{f("a"), f("b")};
            else if (type == "coaxial") fc = fit::Coaxial{f("a"), f("b")};
            else if (type == "radius") fc = fit::Radius{f("label"), s["value"].get<double>()};
            else if (type == "diameter") fc = fit::Diameter{f("label"), s["value"].get<double>()};
            else if (type == "distance") fc = fit::Distance{f("a"), f("b"), s["value"].get<double>()};
            else if (type == "offset") fc = fit::Offset{f("label"), dt(), s["axis"].get<int>(), s["value"].get<double>()};
            else if (type == "axis_distance") fc = fit::AxisDistance{f("a"), f("b"), s["value"].get<double>()};
            if (!fc) continue;
            constraint_of.push_back(c.id);
            in.constraints.push_back(*fc);
        }
        // Given faces follow their datum: they take part as free features held by their constraints.
        const fit::SolveResult r = fit::solve(in);

        d.state_.solved.clear();
        for (std::size_t i = 0; i < label_of.size(); ++i) d.state_.solved[label_of[i]] = r.surfaces[i];
        for (auto& x : d.state_.datums) x.frame = r.datums[static_cast<std::size_t>(datum_index[x.id])];
        // Fillets and holes follow their solved surfaces.
        for (auto& fl : d.state_.fillets)
            if (fl.label && d.state_.solved.contains(fl.label)) {
                if (const auto* c = std::get_if<fit::Cylinder>(&d.state_.solved[fl.label])) fl.measured_radius = c->radius;
            }
        for (auto& h : d.state_.holes) {
            std::optional<fit::Cylinder> axis;
            if (h.wall_label && d.state_.solved.contains(h.wall_label))
                if (const auto* c = std::get_if<fit::Cylinder>(&d.state_.solved[h.wall_label])) {
                    h.measured_diameter = 2 * c->radius;
                    axis = *c;
                }
            if (rim_feature.contains(h.id))
                if (const auto* c = std::get_if<fit::Cylinder>(&r.surfaces[static_cast<std::size_t>(rim_feature[h.id])])) axis = *c;
            if (d.state_.solved.contains(h.host))
                if (const auto* pl = std::get_if<fit::Plane>(&d.state_.solved[h.host])) {
                    // The centre is where the solved axis meets the solved face.
                    if (axis && std::abs(axis->axis.dot(pl->normal)) > 1e-6)
                        h.center = axis->point + (pl->offset - pl->normal.dot(axis->point)) / pl->normal.dot(axis->axis) * axis->axis;
                    else
                        h.center = fit::project(*pl, h.center);
                    h.axis = h.axis.dot(pl->normal) < 0 ? Vec3(-pl->normal) : Vec3(pl->normal);
                }
        }
        json report = {{"converged", r.converged}, {"iterations", r.iterations}, {"constraints", json::object()}, {"labels", json::object()}};
        for (std::size_t i = 0; i < constraint_of.size(); ++i) {
            const auto& c = r.constraints[implicit + i];
            report["constraints"][std::to_string(constraint_of[i])] = {{"status", fit::status_name(c.status)},
                                                                       {"message", c.message},
                                                                       {"violation", c.violation},
                                                                       {"moves_scan_fit_mm", c.max_move_mm},
                                                                       {"rms_change_mm", c.delta_rms_mm}};
        }
        for (std::size_t i = 0; i < label_of.size(); ++i) {
            const auto& f = r.features[i];
            json j = {{"rms_mm", f.rms}, {"moved_from_free_fit_mm", f.max_move_mm}};
            if (f.radius_sd > 0) j["radius_sd_mm"] = f.radius_sd;
            if (f.direction_sd_deg > 0) j["direction_sd_deg"] = f.direction_sd_deg;
            report["labels"][label_name(label_of[i])] = j;
        }
        d.state_.solve_report = report;
        json out = report;
        out["constraints"] = json::array();
        for (const auto& c : d.state_.constraints) out["constraints"].push_back(constraint_json(c));
        return out;
    }

    json build(const json& p) {
        need_scan();
        brep::BuildInput in;
        std::map<int, int> face_of;
        for (const auto& l : d.state_.labels) {
            if (l.role != Role::face) continue;
            const auto s = surface_of(l);
            if (!s) continue;
            brep::FaceInput f{l.name, *s, {}, {}};
            if (!l.given) {
                const fit::RegionPoints pts = fit::region_points(d.topology(), triangles_of(l.id), 2000);
                f.points = pts.points;
                f.normals = pts.normals;
            }
            face_of[l.id] = static_cast<int>(in.faces.size());
            in.faces.push_back(std::move(f));
        }
        if (in.faces.empty()) refuse("no face labels with surfaces: paint and grow faces, or run model.detect");
        for (const auto& h : d.state_.holes) in.holes.push_back({h.name, h.center, h.axis, h.used_diameter(), h.depth});
        for (const auto& f : d.state_.fillets)
            if (face_of.contains(f.face_a) && face_of.contains(f.face_b))
                in.fillets.push_back({f.name, face_of[f.face_a], face_of[f.face_b], f.used_radius()});
        Built b;
        b.result = brep::build(in);
        b.revision = d.revision_ + 1;  // the revision this command produces
        if (b.result.ok) {
            b.tessellation = brep::tessellate(b.result, opt<double>(p, "deflection_mm").value_or(0.02));
            b.bvh = std::make_unique<fit::TriangleBvh>(b.tessellation.mesh);
            fit::DeviationOptions dopt;
            dopt.tolerance_mm = opt<double>(p, "tolerance_mm").value_or(0.1);
            dopt.edge_band_width_mm = 2 * d.voxel_mm();
            b.deviation = fit::scan_to_model(d.mesh(), *b.bvh, b.tessellation.triangle_face, dopt);
            // Deviation per label: each scan vertex takes a label of a triangle it belongs to; ignored scan is left out.
            std::vector<int> vertex_group(d.mesh().vertices.size(), -1);
            const auto& region = *d.state_.region;
            for (std::size_t t = 0; t < region.size(); ++t) {
                if (region[t] == 0) continue;
                const Label* l = d.label(region[t]);
                const int g = l && l->role == Role::ignore ? -2 : region[t];
                for (const auto v : d.mesh().triangles[t]) vertex_group[v] = g;
            }
            b.report = fit::summarise(d.topology(), b.deviation, vertex_group, dopt);
            b.coverage = fit::model_coverage(b.tessellation.mesh, b.tessellation.triangle_face, static_cast<int>(b.result.face_names.size()), d.bvh(), dopt);
        }
        d.built_ = std::move(b);
        const auto& r = d.built_->result;
        json out = {{"ok", r.ok}, {"closed", r.closed}, {"volume_mm3", r.volume}, {"solids", r.solids}, {"faces", r.faces},
                    {"edges", r.edges}, {"scan_coverage", r.scan_coverage}, {"log", r.log}};
        if (r.ok) out["deviation"] = deviation({{"max_hot_spots", 5}});
        return out;
    }

    json export_step(const json& p) {
        if (!d.built_ || !d.built_->result.ok) refuse("no model has been built (model.build)");
        const std::filesystem::path path = get<std::string>(p, "path");
        if (auto r = brep::write_step(d.built_->result, path, d.path_.empty() ? "part" : d.path_.stem().string()); !r) refuse(r.error().message);
        const brep::StepSummary s = brep::read_step(path);
        return {{"path", path.string()}, {"current", d.built_is_current()}, {"read_back", {{"ok", s.ok}, {"valid", s.valid}, {"solids", s.solids},
                                                                                       {"faces", s.faces}, {"volume_mm3", s.volume},
                                                                                       {"named_faces", s.face_names.size()}}}};
    }

    // ---- files ----

    json open(const json& p) {
        const std::filesystem::path path = get<std::string>(p, "path");
        if (path.extension() == ".emodel") {
            if (auto r = d.load(path); !r) refuse(r.error().message);
            return summary();
        }
        auto mesh = load_scan_mesh(path, opt<bool>(p, "fine").value_or(false));
        if (!mesh) refuse(mesh.error().message);
        d.set_scan(std::move(mesh->first), std::move(mesh->second));
        d.path_.clear();
        return summary();
    }

    json open_demo(const json&) {
        const fit::SyntheticPart part = fit::make_synthetic_part(fit::flanged_box_spec());
        d.set_scan(part.mesh, ScanSource{{}, "demo", 0, 0, {}});
        d.path_.clear();
        return summary();
    }

    json save(const json& p) {
        need_scan();
        std::filesystem::path path = opt<std::string>(p, "path").value_or(d.path_.string());
        if (path.empty()) fail("missing parameter 'path' (the document has not been saved before)");
        if (path.extension() != ".emodel") path += ".emodel";
        if (auto r = d.save(path); !r) refuse(r.error().message);
        d.path_ = path;
        return {{"path", path.string()}};
    }
};

namespace {

using Handler = std::function<json(Document::Impl&, const json&)>;

struct CommandInfo {
    Handler handler;
    bool mutating;
};

const std::map<std::string, CommandInfo, std::less<>>& table() {
    using I = Document::Impl;
    static const std::map<std::string, CommandInfo, std::less<>> t = {
        {"summary", {[](I& i, const json&) { return i.summary(); }, false}},
        {"label", {[](I& i, const json& p) { return i.label_info(p); }, false}},
        {"deviation", {[](I& i, const json& p) { return i.deviation(p); }, false}},
        {"raycast", {[](I& i, const json& p) { return i.raycast(p); }, false}},
        {"history", {[](I& i, const json&) { return i.d.history(); }, false}},
        {"open", {[](I& i, const json& p) { return i.open(p); }, true}},
        {"open_demo", {[](I& i, const json& p) { return i.open_demo(p); }, true}},
        {"save", {[](I& i, const json& p) { return i.save(p); }, false}},
        {"label.create", {[](I& i, const json& p) { return i.label_create(p); }, true}},
        {"label.update", {[](I& i, const json& p) { return i.label_update(p); }, true}},
        {"label.delete", {[](I& i, const json& p) { return i.label_delete(p); }, true}},
        {"paint", {[](I& i, const json& p) { return i.paint(p); }, true}},
        {"grow", {[](I& i, const json& p) { return i.grow(p); }, true}},
        {"detect", {[](I& i, const json& p) { return i.detect(p); }, true}},
        {"find_holes", {[](I& i, const json& p) { return i.find_holes(p); }, true}},
        {"hole.update", {[](I& i, const json& p) { return i.hole_update(p); }, true}},
        {"hole.delete", {[](I& i, const json& p) { return i.hole_delete(p); }, true}},
        {"fillet.add", {[](I& i, const json& p) { return i.fillet_add(p); }, true}},
        {"fillet.update", {[](I& i, const json& p) { return i.fillet_update(p); }, true}},
        {"fillet.delete", {[](I& i, const json& p) { return i.fillet_delete(p); }, true}},
        {"datum.create", {[](I& i, const json& p) { return i.datum_create(p); }, true}},
        {"datum.delete", {[](I& i, const json& p) { return i.datum_delete(p); }, true}},
        {"face.add_plane", {[](I& i, const json& p) { return i.face_add_plane(p); }, true}},
        {"constraint.add", {[](I& i, const json& p) { return i.constraint_add(p); }, true}},
        {"constraint.remove", {[](I& i, const json& p) { return i.constraint_remove(p); }, true}},
        {"square", {[](I& i, const json& p) { return i.square(p); }, true}},
        {"solve", {[](I& i, const json& p) { return i.solve(p); }, true}},
        {"build", {[](I& i, const json& p) { return i.build(p); }, true}},
        {"export_step", {[](I& i, const json& p) { return i.export_step(p); }, false}},
    };
    return t;
}

}  // namespace

const std::vector<std::string>& Document::commands() {
    static const std::vector<std::string> names = [] {
        std::vector<std::string> n;
        for (const auto& [k, v] : table()) n.push_back(k);
        for (const char* special : {"undo", "redo", "begin_change", "end_change"}) n.emplace_back(special);
        std::ranges::sort(n);
        return n;
    }();
    return names;
}

Outcome Document::apply(std::string_view command, const json& params, Author author) {
    Outcome out;
    if (command == "undo" || command == "redo") {
        const bool done = command == "undo" ? undo() : redo();
        if (!done) return {false, nullptr, std::format("nothing to {}", command), true};
        out.result = {{"revision", revision_}};
        return out;
    }
    if (command == "begin_change") {
        const std::string why = params.is_object() && params.contains("description") ? params["description"].get<std::string>() : "change";
        begin_change(why, author);
        out.result = {{"depth", change_depth_}};
        return out;
    }
    if (command == "end_change") {
        end_change();
        out.result = {{"depth", change_depth_}, {"revision", revision_}};
        return out;
    }
    const auto it = table().find(command);
    if (it == table().end()) return {false, nullptr, std::format("unknown command '{}'", command), false};
    Impl impl{*this};
    const bool mutating = it->second.mutating;
    const bool opens = command == "open" || command == "open_demo";
    State before = state_;
    try {
        out.result = it->second.handler(impl, params.is_null() ? json::object() : params);
    } catch (const CommandError& e) {
        if (mutating && !opens) state_ = std::move(before);
        return {false, nullptr, e.message, e.refused};
    } catch (const json::exception& e) {
        if (mutating && !opens) state_ = std::move(before);
        return {false, nullptr, std::format("bad parameters: {}", e.what()), false};
    } catch (const std::exception& e) {
        if (mutating && !opens) state_ = std::move(before);
        return {false, nullptr, std::format("failed: {}", e.what()), true};
    } catch (...) {
        if (mutating && !opens) state_ = std::move(before);
        return {false, nullptr, "failed (an unexpected error)", true};
    }
    if (mutating && !opens) {
        // One undo step per command, or one for a whole change (begin_change .. end_change).
        if (change_depth_ == 0) {
            undo_.push_back({std::string(command), author, std::move(before), revision_});
            redo_.clear();
        }
        ++revision_;
    }
    if (out.result.is_object()) out.result["revision"] = revision_;
    return out;
}

}  // namespace einstar::model
