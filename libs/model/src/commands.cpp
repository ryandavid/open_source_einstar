#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <functional>
#include <map>
#include <numbers>
#include <set>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "einstar/fit/detect.hpp"
#include "einstar/fit/grow.hpp"
#include "einstar/fit/holes.hpp"
#include "einstar/fit/primitive_fit.hpp"
#include "einstar/fit/synthetic_part.hpp"
#include "einstar/image/image.hpp"
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
    Author author = Author::user;  // who gave the command

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
        if (l.given) j["given"] = surface_summary_json(*l.given);
        if (l.fit) {
            j["fit"] = surface_summary_json(*l.fit);
            j["sigma_mm"] = l.sigma;
            j["rms_mm"] = l.rms;
        }
        if (const auto it = d.state_.solved.find(l.id); it != d.state_.solved.end()) j["solved"] = surface_summary_json(it->second);
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
                       {"scale_applied", d.source_.scale},
                       {"source", d.source_.status()},
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
        out["photos"] = json::array();
        for (const auto& ph : d.state_.photos) out["photos"].push_back(photo_json(ph, false));
        out["notes"] = json::array();
        for (const auto& n : d.state_.notes) out["notes"].push_back({{"id", n.id}, {"text", n.text}, {"author", n.author}});
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
        if (h.counterbore_diameter) j["counterbore_diameter"] = *h.counterbore_diameter;
        if (h.counterbore_depth) j["counterbore_depth"] = *h.counterbore_depth;
        if (h.countersink_diameter) j["countersink_diameter"] = *h.countersink_diameter;
        if (h.countersink_angle_deg) j["countersink_angle_deg"] = *h.countersink_angle_deg;
        if (h.point_angle_deg) j["point_angle_deg"] = *h.point_angle_deg;
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
        forget_links(id);
        for (const int h : holes_gone) forget_links(h);
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
            if (l.role == Role::ignore || l.given || (l.fit && fit::kind_of(*l.fit) == fit::SurfaceKind::freeform)) continue;
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
            if (g.regions[i].ok) j["fit"] = surface_summary_json(*l.fit);
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
            // Already known: found again, or from the other end of a through hole (on the same axis).
            const auto same_axis = [&](const Hole& h) {
                const Vec3 d0 = c.center - h.center;
                return h.axis.cross(c.axis).norm() < 0.02 && (d0 - d0.dot(h.axis) * h.axis).norm() < 0.25 * std::min(c.diameter, h.used_diameter());
            };
            // A through hole is found from each end; the end with a counterbore or countersink is its entry (the hole
            // keeps its name and constraints).
            const bool formed = c.counterbore_diameter || c.countersink_diameter;
            const auto known = std::ranges::find_if(d.state_.holes, same_axis);
            if (known != d.state_.holes.end() && (!formed || known->counterbore_diameter || known->countersink_diameter)) continue;
            Hole h;
            if (known != d.state_.holes.end()) {
                h.id = known->id;
                h.name = known->name;
                h.diameter = known->diameter;
            } else {
                h.id = new_id();
                h.name = unique_name(std::format("hole {}", d.state_.holes.size() + 1));
            }
            h.host = host_id;
            h.center = h.measured_center = c.center;
            h.axis = c.axis;
            h.measured_diameter = c.diameter;
            h.wall_seen = c.wall_diameter.has_value();
            h.rim = c.opening;
            h.counterbore_diameter = c.counterbore_diameter;
            h.counterbore_depth = c.counterbore_depth;
            h.countersink_diameter = c.countersink_diameter;
            h.countersink_angle_deg = c.countersink_angle_deg;
            h.point_angle_deg = c.point_angle_deg;
            h.seen_depth = c.wall_depth;
            if (c.floor_depth) h.depth = *c.floor_depth;
            // Labels lying (mostly) on the hole's surfaces are part of the hole, not faces of the part: its wall
            // (the cylinder label of the bore's radius), a counterbore's wall and floor, a countersink.
            std::map<int, int> votes;
            for (const auto t : c.wall)
                if (const int r = (*d.state_.region)[t]; r != 0 && r != host_id) ++votes[r];
            std::map<int, int> sizes;
            for (const auto r : *d.state_.region)
                if (votes.contains(r)) ++sizes[r];
            double best_radius_error = 1e9;
            for (const auto& [id, n] : votes) {
                if (2 * n < sizes[id]) continue;
                Label& l = *std::ranges::find(d.state_.labels, id, &Label::id);
                l.role = Role::hole;
                if (const auto* cyl = l.fit ? std::get_if<fit::Cylinder>(&*l.fit) : nullptr) {
                    const double e = std::abs(2 * cyl->radius - c.diameter);
                    if (e < best_radius_error && e < 0.25 * c.diameter) {
                        best_radius_error = e;
                        h.wall_label = id;
                    }
                }
            }
            if (!h.wall_label && known != d.state_.holes.end()) h.wall_label = known->wall_label;
            if (known != d.state_.holes.end()) {
                *known = h;
                out.push_back(hole_json(h));
                continue;
            }
            d.state_.holes.push_back(h);
            out.push_back(hole_json(h));
        }
        return out;
    }

    // A freeform face on a label: fitted to its region (or its paint). With `extend` (default when it has no
    // region yet) the region first takes in the unlabelled scan connected to it up to sharp creases, so "the rest of
    // this curved top" is one stroke and one command.
    json freeform(const json& p) {
        need_scan();
        Label& l = label_ref(need(p, "label"));
        auto tris = triangles_of(l.id);
        const bool extend = opt<bool>(p, "extend").value_or(tris.empty());
        if (tris.empty()) tris = triangles_of(l.id, true);
        if (tris.empty()) refuse(std::format("'{}' has no paint or region: paint a few mm of the face first", l.name));
        auto region = region_copy();
        auto paint = paint_copy();
        if (extend) {
            // Stops at creases, and where the surface turns more than 70 degrees from the seed's own facing (a
            // freeform face is a height field; the scan rounds sharp edges over a few triangles, each step small).
            const double crease = std::cos(opt<double>(p, "crease_deg").value_or(20.0) * std::numbers::pi / 180);
            const auto& topo = d.topology();
            Vec3f facing = Vec3f::Zero();
            for (const auto t : tris) facing += topo.area(t) * topo.normal(t);
            facing.normalize();
            const float max_tilt = std::cos(70.0f * std::numbers::pi_v<float> / 180);
            std::vector<std::uint8_t> in(region.size(), 0);
            for (const auto t : tris) in[t] = 1;
            std::vector<std::uint32_t> stack(tris.begin(), tris.end());
            while (!stack.empty()) {
                const auto t = stack.back();
                stack.pop_back();
                for (const auto n : topo.neighbors(t)) {
                    if (n == fit::kNoTriangle || in[n] || (region[n] != 0 && region[n] != l.id) || (paint[n] != 0 && paint[n] != l.id)) continue;
                    if (topo.normal(t).dot(topo.normal(n)) < crease || topo.normal(n).dot(facing) < max_tilt) continue;
                    in[n] = 1;
                    tris.push_back(n);
                    stack.push_back(n);
                }
            }
            std::ranges::sort(tris);
        }
        const fit::RegionPoints pts = fit::region_points(d.topology(), tris, 30000);
        fit::FreeformOptions fo;
        if (const auto sp = opt<double>(p, "spacing_mm")) fo.spacing_mm = *sp;
        if (const auto sm = opt<double>(p, "smoothness")) fo.smoothness = *sm;
        std::string why;
        const auto r = fit::fit_freeform(pts.view(), fo, &why);
        if (!r) refuse(why);
        for (const auto t : tris) region[t] = static_cast<std::uint16_t>(l.id);
        set_region(std::move(region));
        l.fit = r->surface;
        l.sigma = r->sigma;
        l.rms = r->rms;
        l.role = Role::face;
        l.kinds = {fit::SurfaceKind::freeform};
        d.state_.solved.erase(l.id);
        return {{"label", l.name}, {"triangles", tris.size()}, {"fit", surface_summary_json(r->surface)}, {"sigma_mm", r->sigma}, {"rms_mm", r->rms}};
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
        // Forms: a number sets one, null removes it.
        const auto form = [&](const char* key, std::optional<double>& field) {
            if (!p.contains(key)) return;
            if (p[key].is_null()) {
                field.reset();
                return;
            }
            const auto x = get<double>(p, key);
            if (!(x > 0)) fail(std::format("{} must be positive", key));
            field = x;
        };
        form("counterbore_diameter", h.counterbore_diameter);
        form("counterbore_depth", h.counterbore_depth);
        form("countersink_diameter", h.countersink_diameter);
        form("countersink_angle_deg", h.countersink_angle_deg);
        form("point_angle_deg", h.point_angle_deg);
        if (h.counterbore_diameter.has_value() != h.counterbore_depth.has_value()) fail("a counterbore needs both its diameter and its depth");
        if (h.countersink_diameter && !h.countersink_angle_deg) h.countersink_angle_deg = 90.0;
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
        forget_links(id);
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
        forget_links(id);
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
        } else if (type == "tangent" || type == "equal_radius") {
            lbl("a");
            lbl("b");
        } else if (type == "symmetric") {
            any("a");
            any("b");
            spec["datum"] = datum_ref(need(p, "datum")).id;
            spec["axis"] = axis_index(need(p, "axis"));
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

    // ---- photos and notes (photo.hpp) ----

    std::string author_text() const { return std::string(author_name(author)); }

    Photo& photo_ref(const json& ref) {
        for (auto& ph : d.state_.photos)
            if ((ref.is_number_integer() && ph.id == ref.get<int>()) || (ref.is_string() && ph.name == ref.get<std::string>())) return ph;
        fail(std::format("no photo {}", ref.dump()));
    }
    std::pair<Photo*, Annotation*> annotation_ref(const json& ref) {
        for (auto& ph : d.state_.photos)
            for (auto& a : ph.annotations)
                if ((ref.is_number_integer() && a.id == ref.get<int>()) || (ref.is_string() && a.name == ref.get<std::string>())) return {&ph, &a};
        fail(std::format("no annotation {}", ref.dump()));
    }
    Note& note_ref(const json& ref) {
        for (auto& n : d.state_.notes)
            if (ref.is_number_integer() && n.id == ref.get<int>()) return n;
        fail(std::format("no note {} (notes are referred to by id)", ref.dump()));
    }
    // What an annotation can be about: a label, a hole or a fillet.
    int link_ref(const json& ref) {
        if (const Label* l = d.find_label(ref)) return l->id;
        for (const auto& h : d.state_.holes)
            if ((ref.is_number_integer() && h.id == ref.get<int>()) || (ref.is_string() && h.name == ref.get<std::string>())) return h.id;
        for (const auto& f : d.state_.fillets)
            if ((ref.is_number_integer() && f.id == ref.get<int>()) || (ref.is_string() && f.name == ref.get<std::string>())) return f.id;
        fail(std::format("no label, hole or fillet {}", ref.dump()));
    }
    std::string link_name(int id) const {
        for (const auto& f : d.state_.fillets)
            if (f.id == id) return f.name;
        return label_name(id);
    }
    void forget_links(int id) {
        for (auto& ph : d.state_.photos)
            for (auto& a : ph.annotations) std::erase(a.links, id);
    }

    json annotation_json(const Photo& ph, const Annotation& a) const {
        json j = {{"id", a.id}, {"name", a.name}, {"photo", ph.name}, {"kind", annotation_kind_name(a.kind)}, {"points", json::array()},
                  {"summary", annotation_summary(a)}, {"author", a.author}};
        for (const auto& q : a.points) j["points"].push_back({std::round(q.x() * 10) / 10, std::round(q.y() * 10) / 10});
        if (a.value) j["value"] = *a.value;
        if (a.value) j["unit"] = a.kind == AnnotationKind::angle ? "deg" : "mm";
        if (!a.entered.empty()) j["entered"] = a.entered;
        if (a.tolerance) j["tolerance"] = *a.tolerance;
        if (!a.text.empty()) j["text"] = a.text;
        if (!a.links.empty()) {
            j["links"] = json::array();
            for (const int id : a.links) j["links"].push_back(link_name(id));
        }
        if (const auto c = annotation_circle(a)) j["circle"] = {{"center", {c->first.x(), c->first.y()}}, {"radius_px", c->second}};
        if (a.applied) j["applied_to"] = a.applied;
        return j;
    }
    json photo_json(const Photo& ph, bool annotations) const {
        json j = {{"id", ph.id}, {"name", ph.name}, {"size", {ph.width, ph.height}}, {"mime", ph.blob ? ph.blob->mime : ""},
                  {"bytes", ph.blob ? ph.blob->bytes.size() : 0}, {"annotation_count", ph.annotations.size()}, {"registered", ph.camera.has_value()}};
        if (!ph.caption.empty()) j["caption"] = ph.caption;
        if (!ph.exif.empty()) j["exif"] = ph.exif;
        if (annotations) {
            j["annotations"] = json::array();
            for (const auto& a : ph.annotations) j["annotations"].push_back(annotation_json(ph, a));
        } else {
            j["annotations"] = json::array();
            for (const auto& a : ph.annotations) j["annotations"].push_back(annotation_summary(a));
        }
        if (!ph.correspondences.empty()) j["matched_points"] = ph.correspondences.size();
        if (ph.camera) j["camera"] = {{"rms_px", ph.camera->rms_px}, {"focal_px", ph.camera->focal_px}};
        return j;
    }

    std::string unique_photo_name(const std::string& base) const {
        const auto taken = [&](const std::string& n) { return std::ranges::any_of(d.state_.photos, [&](const Photo& ph) { return ph.name == n; }); };
        if (!taken(base)) return base;
        for (int i = 2;; ++i)
            if (const auto n = std::format("{} {}", base, i); !taken(n)) return n;
    }
    std::string next_annotation_name(AnnotationKind kind) const {
        int n = 0;
        const std::string prefix(annotation_prefix(kind));
        for (const auto& ph : d.state_.photos)
            for (const auto& a : ph.annotations)
                if (a.name.starts_with(prefix) && a.name.size() > prefix.size() && std::isdigit(static_cast<unsigned char>(a.name[prefix.size()])))
                    n = std::max(n, std::atoi(a.name.c_str() + prefix.size()));
        return std::format("{}{}", prefix, n + 1);
    }

    // Adds a photo from a file's bytes (kept as they are).
    Photo& add_photo(std::string bytes, const std::string& name, const std::string& caption) {
        const auto info = image::probe(bytes);
        if (!info) refuse(std::format("{}: {}", name, info.error().message));
        Photo ph;
        ph.id = new_id();
        ph.name = unique_photo_name(name.empty() ? "photo" : name);
        ph.caption = caption;
        ph.width = info->width;
        ph.height = info->height;
        const auto& e = info->exif;
        if (!e.make.empty()) ph.exif["make"] = e.make;
        if (!e.model.empty()) ph.exif["model"] = e.model;
        if (!e.taken.empty()) ph.exif["taken"] = e.taken;
        if (e.focal_mm) ph.exif["focal_mm"] = *e.focal_mm;
        if (e.focal_35mm) ph.exif["focal_35mm"] = *e.focal_35mm;
        const std::string hash = image::content_hash(bytes);
        for (const auto& other : d.state_.photos)
            if (other.blob && other.blob->hash == hash) ph.blob = other.blob;  // the same file again: shared
        if (!ph.blob) ph.blob = std::make_shared<const PhotoBlob>(PhotoBlob{std::move(bytes), info->mime, hash});
        d.state_.photos.push_back(std::move(ph));
        return d.state_.photos.back();
    }

    json photo_import(const json& p) {
        need_scan();
        std::vector<std::filesystem::path> paths;
        if (p.contains("path")) paths.emplace_back(get<std::string>(p, "path"));
        if (p.contains("paths"))
            for (const auto& x : need(p, "paths")) paths.emplace_back(x.get<std::string>());
        const std::string caption = opt<std::string>(p, "caption").value_or("");
        json out = json::array();
        for (const auto& path : paths) {
            std::ifstream f(path, std::ios::binary);
            if (!f) refuse("cannot read " + path.string());
            std::string bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
            out.push_back(photo_json(add_photo(std::move(bytes), opt<std::string>(p, "name").value_or(path.stem().string()), caption), false));
        }
        if (const auto data = opt<std::string>(p, "data_base64")) {
            auto bytes = base64_decode(*data);
            if (!bytes) fail("data_base64 is not base64");
            out.push_back(photo_json(add_photo(std::move(*bytes), opt<std::string>(p, "name").value_or("photo"), caption), false));
        }
        if (out.empty()) fail("give 'path', 'paths' or 'data_base64'");
        return {{"photos", out}};
    }

    json photo_update(const json& p) {
        Photo& ph = photo_ref(need(p, "photo"));
        if (const auto n = opt<std::string>(p, "name"); n && *n != ph.name) {
            if (n->empty() || std::ranges::any_of(d.state_.photos, [&](const Photo& o) { return o.name == *n; }))
                fail(std::format("the photo name '{}' is taken or empty", *n));
            ph.name = *n;
        }
        if (const auto c = opt<std::string>(p, "caption")) ph.caption = *c;
        if (const auto u = opt<bool>(p, "use_for_colour")) ph.use_for_colour = *u;
        return photo_json(ph, false);
    }

    json photo_delete(const json& p) {
        const int id = photo_ref(need(p, "photo")).id;
        std::erase_if(d.state_.photos, [&](const Photo& ph) { return ph.id == id; });
        return {{"deleted", id}};
    }

    std::vector<Vec2> points_param(const json& p, const Photo& ph, AnnotationKind kind) {
        std::vector<Vec2> pts;
        for (const auto& q : need(p, "points")) {
            if (!q.is_array() || q.size() != 2) fail("a point is [x, y] in photo pixels");
            const Vec2 v(q[0].get<double>(), q[1].get<double>());
            if (v.x() < 0 || v.y() < 0 || v.x() > ph.width || v.y() > ph.height)
                fail(std::format("the point [{:.0f}, {:.0f}] is outside the photo ({} x {} px)", v.x(), v.y(), ph.width, ph.height));
            pts.push_back(v);
        }
        const auto [lo, hi] = annotation_points(kind);
        if (static_cast<int>(pts.size()) < lo || static_cast<int>(pts.size()) > hi)
            fail(std::format("a {} takes {} point{}", annotation_kind_name(kind), lo == hi ? std::to_string(lo) : std::format("{} or {}", lo, hi),
                             hi > 1 ? "s" : ""));
        return pts;
    }
    // A value given as a number (mm / degrees) or as typed ("1 1/2 in", "Ø6 ±0.02", "30°").
    void set_value(Annotation& a, const json& v) {
        if (v.is_null()) {
            a.value.reset();
            a.entered.clear();
            a.tolerance.reset();
            return;
        }
        if (v.is_number()) {
            a.value = v.get<double>();
            a.entered.clear();
            return;
        }
        if (!v.is_string()) fail("a value is a number or text such as '42 mm', '1 1/2 in', 'Ø6', '30°'");
        const auto parsed = parse_value(v.get<std::string>(), a.kind == AnnotationKind::angle);
        if (!parsed) fail(std::format("cannot read '{}' as a value (e.g. 42, 42 mm, 1.5 in, 1 1/2\", Ø6, R2, 30°)", v.get<std::string>()));
        if (parsed->angle != (a.kind == AnnotationKind::angle))
            fail(a.kind == AnnotationKind::angle ? "an angle's value is in degrees" : "a length's value cannot be an angle");
        a.value = parsed->value;
        if (a.kind == AnnotationKind::diameter && parsed->form == 'R') *a.value *= 2;  // "R3" on a diameter: 6
        a.entered = v.get<std::string>();
        if (parsed->tolerance) a.tolerance = parsed->tolerance;
    }

    json photo_annotate(const json& p) {
        Photo& ph = photo_ref(need(p, "photo"));
        const auto kind = annotation_kind_from_name(get<std::string>(p, "kind"));
        if (!kind) fail("kind is dimension, diameter, angle, callout or note");
        Annotation a;
        a.kind = *kind;
        a.points = points_param(p, ph, a.kind);
        if (p.contains("value")) set_value(a, p["value"]);
        if (const auto t = opt<double>(p, "tolerance")) a.tolerance = *t;
        a.text = opt<std::string>(p, "text").value_or("");
        if (p.contains("links"))
            for (const auto& ref : p["links"]) a.links.push_back(link_ref(ref));
        if ((a.kind == AnnotationKind::callout || a.kind == AnnotationKind::note) && !a.value && a.text.empty())
            fail(std::format("a {} needs text or a value", annotation_kind_name(a.kind)));
        a.id = new_id();
        a.name = next_annotation_name(a.kind);
        a.author = author_text();
        ph.annotations.push_back(a);
        return annotation_json(ph, ph.annotations.back());
    }

    json photo_annotation_update(const json& p) {
        auto [ph, a] = annotation_ref(need(p, "annotation"));
        if (p.contains("points")) a->points = points_param(p, *ph, a->kind);
        if (p.contains("value")) set_value(*a, p["value"]);
        if (p.contains("tolerance")) a->tolerance = p["tolerance"].is_null() ? std::nullopt : std::optional<double>(get<double>(p, "tolerance"));
        if (const auto t = opt<std::string>(p, "text")) a->text = *t;
        if (p.contains("links")) {
            a->links.clear();
            for (const auto& ref : p["links"]) a->links.push_back(link_ref(ref));
        }
        return annotation_json(*ph, *a);
    }

    json photo_annotation_delete(const json& p) {
        auto [ph, a] = annotation_ref(need(p, "annotation"));
        const int id = a->id;
        std::erase_if(ph->annotations, [&](const Annotation& x) { return x.id == id; });
        return {{"deleted", id}};
    }

    json photo_list(const json& p) const {
        json out = json::array();
        const auto only = p.is_object() && p.contains("photo") ? std::optional<json>(p["photo"]) : std::nullopt;
        for (const auto& ph : d.state_.photos) {
            if (only && !((only->is_number_integer() && ph.id == only->get<int>()) || (only->is_string() && ph.name == only->get<std::string>()))) continue;
            out.push_back(photo_json(ph, true));
        }
        if (only && out.empty()) fail(std::format("no photo {}", only->dump()));
        json notes = json::array();
        for (const auto& n : d.state_.notes) notes.push_back({{"id", n.id}, {"text", n.text}, {"author", n.author}});
        return {{"photos", out}, {"notes", notes}};
    }

    json note_add(const json& p) {
        const auto text = get<std::string>(p, "text");
        if (text.empty()) fail("a note needs text");
        d.state_.notes.push_back({new_id(), text, author_text()});
        return {{"id", d.state_.notes.back().id}, {"text", text}};
    }
    json note_update(const json& p) {
        Note& n = note_ref(need(p, "note"));
        n.text = get<std::string>(p, "text");
        return {{"id", n.id}, {"text", n.text}};
    }
    json note_delete(const json& p) {
        const int id = note_ref(need(p, "note")).id;
        std::erase_if(d.state_.notes, [&](const Note& n) { return n.id == id; });
        return {{"deleted", id}};
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
            else if (type == "tangent") fc = fit::Tangent{f("a"), f("b")};
            else if (type == "equal_radius") fc = fit::EqualRadius{f("a"), f("b")};
            else if (type == "symmetric") fc = fit::Symmetric{f("a"), f("b"), dt(), s["axis"].get<int>()};
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
        report["scale"] = scale_check();
        d.state_.solve_report = report;
        json out = report;
        out["constraints"] = json::array();
        for (const auto& c : d.state_.constraints) out["constraints"].push_back(constraint_json(c));
        return out;
    }

    // What the user's measurements say about the scan's scale: each measured length against what the scan alone
    // gives (free fits, not the solved surfaces the measurements were forced onto), and the least-squares factor of
    // all of them. Long lengths weigh most; small diameters carry the mesher's slight bias.
    json scale_check() const {
        struct Pair {
            int constraint;
            double measured, scanned;
        };
        std::vector<Pair> pairs;
        const auto fitted = [&](int id) -> std::optional<fit::Surface> {
            if (const Label* l = d.label(id)) return l->fit;
            for (const auto& h : d.state_.holes)
                if (h.id == id) {
                    if (const Label* wall = d.label(h.wall_label); wall && wall->fit) return wall->fit;
                    return fit::Surface(fit::Cylinder{h.measured_center, h.axis, 0.5 * h.measured_diameter});
                }
            return std::nullopt;
        };
        for (const auto& c : d.state_.constraints) {
            const auto& s = c.spec;
            const std::string type = s["type"];
            if (!s.contains("value")) continue;
            const double m = s["value"].get<double>();
            if (type == "diameter" && s.contains("hole")) {
                for (const auto& h : d.state_.holes)
                    if (h.id == s["hole"].get<int>()) pairs.push_back({c.id, m, h.measured_diameter});
            } else if (type == "diameter" || type == "radius") {
                const auto f = fitted(s["label"].get<int>());
                const auto* cyl = f ? std::get_if<fit::Cylinder>(&*f) : nullptr;
                if (cyl) pairs.push_back({c.id, m, (type == "diameter" ? 2 : 1) * cyl->radius});
            } else if (type == "distance") {
                const auto a = fitted(s["a"].get<int>()), b = fitted(s["b"].get<int>());
                if (a && b && std::holds_alternative<fit::Plane>(*a) && std::holds_alternative<fit::Plane>(*b)) {
                    const auto& pa = std::get<fit::Plane>(*a);
                    pairs.push_back({c.id, m, std::abs(pa.normal.dot(fit::position_of(*b)) - pa.offset)});
                }
            } else if (type == "axis_distance") {
                const auto a = fitted(s["a"].get<int>()), b = fitted(s["b"].get<int>());
                const auto da = a ? fit::direction_of(*a) : std::nullopt;
                if (a && b && da) {
                    Vec3 v = fit::position_of(*b) - fit::position_of(*a);
                    v -= v.dot(*da) * *da;
                    pairs.push_back({c.id, m, v.norm()});
                }
            }
        }
        if (pairs.empty()) return nullptr;
        double ms = 0, ss = 0;
        for (const auto& p : pairs) {
            ms += p.measured * p.scanned;
            ss += p.scanned * p.scanned;
        }
        const double k = ms / ss;
        double res = 0;
        for (const auto& p : pairs) res += std::pow(p.measured - k * p.scanned, 2);
        const double sd = pairs.size() > 1 ? std::sqrt(res / static_cast<double>(pairs.size() - 1) / ss) : 0.0;
        json out = {{"factor", k}, {"percent", 100 * (k - 1)}, {"sd_percent", 100 * sd}, {"measurements", json::array()}};
        for (const auto& p : pairs)
            out["measurements"].push_back({{"constraint", p.constraint}, {"measured", p.measured}, {"scanned", p.scanned},
                                           {"implied_factor", p.measured / p.scanned}});
        const bool significant = pairs.size() >= 2 && std::abs(k - 1) > std::max(3 * sd, 5e-4);
        out["significant"] = significant;
        out["advice"] = significant ? std::format("the measurements say the scan is {:.2f}% {}; model.scale {{\"factor\": {:.5f}}} corrects it "
                                                  "(and suggests checking the scanner's calibration)",
                                                  std::abs(100 * (k - 1)), k < 1 ? "large" : "small", k)
                                    : std::string("the measurements agree with the scan's scale");
        return out;
    }

    json scale(const json& p) {
        need_scan();
        const double k = get<double>(p, "factor");
        if (!(k > 0.9 && k < 1.1)) fail("a scale correction is a factor between 0.9 and 1.1");
        d.scale_scan(k);
        return {{"scale", d.source_.scale}, {"note", "the scan and everything on it were scaled; the undo history was cleared"}};
    }

    // Makes the mesh again from the scan's source (an .estr that has since been continued or edited, or a
    // re-exported mesh), and carries the modelling over: each new triangle takes the label of the old triangle
    // nearest to it (within two voxels, facing the same way), holes' openings move to the nearest new vertices,
    // and every fitted face is refitted to its carried region. The undo history is cleared.
    json reprocess(const json& p) {
        need_scan();
        const std::filesystem::path path = opt<std::string>(p, "path").value_or(d.source_.path.string());
        if (path.empty()) refuse("this scan has no source file to process again (the demo part)");
        const double voxel_before = d.source_.process.is_object() ? d.source_.process.value("voxel_mm", 0.5) : 0.5;
        const bool fine = opt<bool>(p, "fine").value_or(voxel_before < 0.4);
        auto loaded = load_scan_mesh(path, fine);
        if (!loaded) refuse(loaded.error().message);
        auto& [mesh, source] = *loaded;
        // The modelling is in the scaled scan's frame (model.scale).
        const double k = d.source_.scale;
        if (k != 1.0)
            for (auto& v : mesh.vertices) v *= static_cast<float>(k);
        source.scale = k;

        // The old scan, kept while the modelling moves across.
        auto old_mesh = std::move(d.mesh_);
        auto old_bvh = std::move(d.bvh_);
        auto old_topo = std::move(d.topo_);
        const auto old_paint = d.state_.paint;
        const auto old_region = d.state_.region;
        const float reach = static_cast<float>(2 * std::max(d.voxel_mm_, 0.1));
        d.adopt_mesh(std::move(mesh));
        d.source_ = std::move(source);

        const auto& nm = d.mesh();
        std::vector<std::uint16_t> paint(nm.triangles.size(), 0), region(nm.triangles.size(), 0);
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, nm.triangles.size(), 4096), [&](const auto& r) {
            for (std::size_t t = r.begin(); t < r.end(); ++t) {
                const auto u = static_cast<std::uint32_t>(t);
                const auto hit = old_bvh->closest(d.topology().centroid(u), reach);
                if (!hit || d.topology().normal(u).dot(old_topo->normal(hit->triangle)) < 0.5f) continue;
                paint[t] = (*old_paint)[hit->triangle];
                region[t] = (*old_region)[hit->triangle];
            }
        });
        std::size_t carried = 0;
        for (const auto r : region) carried += r != 0;
        set_paint(std::move(paint));
        set_region(std::move(region));

        // Holes' openings: the nearest new vertex to each old one.
        for (auto& h : d.state_.holes) {
            std::vector<std::uint32_t> rim;
            for (const auto v : h.rim) {
                const Vec3f q = old_mesh->vertices[v];
                const auto hit = d.bvh().closest(q, reach);
                if (!hit) continue;
                const auto& tri = nm.triangles[hit->triangle];
                std::uint32_t best = tri[0];
                for (const auto c : tri)
                    if ((nm.vertices[c] - q).squaredNorm() < (nm.vertices[best] - q).squaredNorm()) best = c;
                if (std::ranges::find(rim, best) == rim.end()) rim.push_back(best);
            }
            h.rim = std::move(rim);
        }

        // Refit each fitted face to its region on the new scan.
        json labels = json::array();
        for (auto& l : d.state_.labels) {
            if (l.given || !l.fit) continue;
            const auto tris = triangles_of(l.id);
            json j = {{"label", l.name}, {"triangles", tris.size()}};
            if (fit::kind_of(*l.fit) != fit::SurfaceKind::freeform && tris.size() >= 12) {
                const fit::RegionPoints pts = fit::region_points(d.topology(), tris, 20000);
                const auto r = fit::refine_surface(*l.fit, pts.view());
                double moved = 0;  // how far the face moved, over its region
                for (const auto& q : pts.points) moved = std::max(moved, std::abs(fit::signed_distance(r.surface, fit::project(*l.fit, q))));
                j["moved_mm"] = moved;
                l.fit = r.surface;
                l.sigma = r.sigma;
                l.rms = r.rms;
                j["rms"] = r.rms;
            } else {
                j["refit"] = false;  // freeform, or too little of it carried over: kept as it was
            }
            labels.push_back(j);
        }
        d.state_.solved.clear();
        d.state_.solve_report = json();
        d.undo_.clear();
        d.redo_.clear();
        d.change_depth_ = 0;
        d.built_.reset();
        ++d.revision_;
        return {{"scan", summary()["scan"]},
                {"carried_triangles", carried},
                {"labels", labels},
                {"note", "the labels were carried to the new scan and refitted; solve and build again. The undo history was cleared"}};
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
        for (const auto& h : d.state_.holes)
            in.holes.push_back({h.name, h.center, h.axis, h.used_diameter(), h.depth, h.counterbore_diameter, h.counterbore_depth, h.countersink_diameter,
                                h.countersink_angle_deg, h.point_angle_deg});
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
        {"freeform", {[](I& i, const json& p) { return i.freeform(p); }, true}},
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
        {"scale", {[](I& i, const json& p) { return i.scale(p); }, false}},
        {"reprocess", {[](I& i, const json& p) { return i.reprocess(p); }, false}},
        {"export_step", {[](I& i, const json& p) { return i.export_step(p); }, false}},
        {"photo.import", {[](I& i, const json& p) { return i.photo_import(p); }, true}},
        {"photo.update", {[](I& i, const json& p) { return i.photo_update(p); }, true}},
        {"photo.delete", {[](I& i, const json& p) { return i.photo_delete(p); }, true}},
        {"photo.annotate", {[](I& i, const json& p) { return i.photo_annotate(p); }, true}},
        {"photo.annotation.update", {[](I& i, const json& p) { return i.photo_annotation_update(p); }, true}},
        {"photo.annotation.delete", {[](I& i, const json& p) { return i.photo_annotation_delete(p); }, true}},
        {"photo.list", {[](I& i, const json& p) { return i.photo_list(p); }, false}},
        {"note.add", {[](I& i, const json& p) { return i.note_add(p); }, true}},
        {"note.update", {[](I& i, const json& p) { return i.note_update(p); }, true}},
        {"note.delete", {[](I& i, const json& p) { return i.note_delete(p); }, true}},
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
    Impl impl{*this, author};
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
