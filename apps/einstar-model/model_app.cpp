#include "model_app.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

#include "imgui.h"

#include "dialogs.hpp"
#include "einstar/model/io.hpp"
#include "einstar/model/json.hpp"
#include "einstar/model/photo_geometry.hpp"

namespace einstar::modelapp {
namespace {

constexpr std::array<const char*, 4> kDisplayNames{"labels", "deviation", "model", "photos"};
constexpr std::array<const char*, 14> kConstraintTypes{"aligned",   "parallel", "perpendicular", "angle",         "coplanar",
                                                       "coaxial",   "radius",   "diameter",      "distance",      "offset",
                                                       "axis_distance", "tangent", "symmetric", "equal_radius"};

render::Rgba8 hsv(float h, float s, float v) {
    const float c = v * s, x = c * (1 - std::abs(std::fmod(h * 6, 2.0f) - 1)), m = v - c;
    float r = 0, g = 0, b = 0;
    switch (static_cast<int>(h * 6) % 6) {
        case 0: r = c, g = x; break;
        case 1: r = x, g = c; break;
        case 2: g = c, b = x; break;
        case 3: g = x, b = c; break;
        case 4: r = x, b = c; break;
        default: r = c, b = x; break;
    }
    return {static_cast<std::uint8_t>(255 * (r + m)), static_cast<std::uint8_t>(255 * (g + m)), static_cast<std::uint8_t>(255 * (b + m)), 255};
}

ImVec4 im(const render::Rgba8& c) { return {c.r / 255.0f, c.g / 255.0f, c.b / 255.0f, 1.0f}; }

ImVec4 status_color(const std::string& status) {
    if (status == "satisfied") return {0.4f, 0.85f, 0.45f, 1};
    if (status == "redundant") return {0.75f, 0.75f, 0.75f, 1};
    if (status == "conflict") return {1.0f, 0.35f, 0.3f, 1};
    if (status == "invalid") return {1.0f, 0.6f, 0.2f, 1};
    return {0.6f, 0.6f, 0.6f, 1};
}

// Names of the labels (for combos), with a leading "-" entry when `none` is wanted.
std::vector<std::string> label_names(const model::Document& doc) {
    std::vector<std::string> n;
    for (const auto& l : doc.state().labels) n.push_back(l.name);
    return n;
}

bool combo(const char* id, int& index, const std::vector<std::string>& items) {
    bool changed = false;
    const char* preview = index >= 0 && index < static_cast<int>(items.size()) ? items[static_cast<std::size_t>(index)].c_str() : "-";
    if (ImGui::BeginCombo(id, preview)) {
        for (int i = 0; i < static_cast<int>(items.size()); ++i)
            if (ImGui::Selectable(items[static_cast<std::size_t>(i)].c_str(), i == index)) {
                index = i;
                changed = true;
            }
        ImGui::EndCombo();
    }
    return changed;
}

// An up direction for a view along `forward`: the scan's -y (its images' up) unless that is nearly the view.
Vec3 any_up(const Vec3& forward) {
    const Vec3 up = -Vec3::UnitY();
    return std::abs(up.dot(forward)) < 0.95 ? up : Vec3(Vec3::UnitZ());
}

}  // namespace

std::string_view display_name(Display d) { return kDisplayNames[static_cast<std::size_t>(d)]; }
std::optional<Display> display_from_name(std::string_view s) {
    for (std::size_t i = 0; i < kDisplayNames.size(); ++i)
        if (s == kDisplayNames[i]) return static_cast<Display>(i);
    return std::nullopt;
}

ModelApp::ModelApp() : settings(model::load_settings()) {}

model::Outcome ModelApp::run(std::string_view command, const json& params, model::Author author) {
    model::Outcome o = doc.apply(command, params, author);
    if (o.ok) {
        status_ = std::format("{}: done", command);
        status_error_ = false;
    } else {
        status_ = std::format("{}: {}", command, o.error);
        status_error_ = true;
    }
    if (o.ok && (command == "save" || command == "open") && params.contains("path")) {
        model::remember_recent(settings, params["path"].get<std::string>());
        (void)model::save_settings(settings);
    }
    return o;
}

void ModelApp::open(const std::filesystem::path& path, bool fine) {
    if (busy()) return;
    if (path.extension() == ".emodel") {
        run("open", {{"path", path.string()}});
        return;
    }
    job_path_ = path;
    busy_text_ = path.extension() == ".estr" ? std::format("Processing {} ...", path.filename().string()) : std::format("Reading {} ...", path.filename().string());
    job_ = std::async(std::launch::async, [path, fine] { return model::load_scan_mesh(path, fine); });
}

bool ModelApp::is_slow(std::string_view command) {
    for (const char* c : {"detect", "grow", "solve", "build", "find_holes", "export_step", "freeform", "reprocess"})
        if (command == c) return true;
    return false;
}

bool ModelApp::start(std::string_view command, json params, model::Author author) {
    if (busy()) return false;
    if (!is_slow(command)) {
        open_outcome_ = run(command, params, author);
        return true;
    }
    command_name_ = std::string(command);
    busy_text_ = command == "detect" ? "Detecting faces ..." : command == "build" ? "Building the solid ..." : command == "reprocess" ? "Processing the scan again ..." : std::format("{} ...", command);
    command_job_ = std::async(std::launch::async, [this, c = command_name_, p = std::move(params), author] { return doc.apply(c, p, author); });
    return true;
}

std::string ModelApp::photo_colours_key() const {
    std::string key = std::format("{}", static_cast<const void*>(doc.has_scan() ? &doc.mesh() : nullptr));
    for (const auto& ph : doc.state().photos)
        if (ph.camera && ph.use_for_colour) key += std::format("|{}:{:.6f}:{:.6f}", ph.id, ph.camera->focal_px, ph.camera->T_camera_world.translation().x());
    // Background labels (ignore) are left uncoloured: a change of role changes the colouring.
    for (const auto& l : doc.state().labels)
        if (l.role == model::Role::ignore) key += std::format("|i{}", l.id);
    return key;
}

void ModelApp::update() {
    if (colour_job_.valid() && colour_job_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        photo_colours_ = colour_job_.get();
        busy_text_.clear();
        shown_revision_ = ~0ull;  // redraw with them
        return;
    }
    // Photo colours wanted and out of date: made in the background (the document waits).
    if (display == Display::photos && !busy() && doc.has_scan()) {
        const auto key = photo_colours_key();
        if (key != photo_colours_key_) {
            photo_colours_key_ = key;
            busy_text_ = "Colouring the scan from the photos ...";
            colour_job_ = std::async(std::launch::async, [this] { return model::photo_colours(doc); });
            return;
        }
    }
    if (command_job_.valid() && command_job_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        model::Outcome o = command_job_.get();
        busy_text_.clear();
        status_ = o.ok ? std::format("{}: done", command_name_) : std::format("{}: {}", command_name_, o.error);
        status_error_ = !o.ok;
        if (o.ok && command_name_ == "build" && display == Display::labels) display = Display::model;
        open_outcome_ = std::move(o);
        return;
    }
    if (!job_.valid() || job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    auto r = job_.get();
    busy_text_.clear();
    if (!r) {
        open_outcome_ = model::Outcome{false, nullptr, r.error().message, true};
        status_ = std::format("open: {}", r.error().message);
        status_error_ = true;
        return;
    }
    doc.set_scan(std::move(r->first), std::move(r->second));
    model::remember_recent(settings, job_path_);
    (void)model::save_settings(settings);
    status_ = std::format("opened {}", job_path_.filename().string());
    status_error_ = false;
    open_outcome_ = model::Outcome{true, doc.apply("summary", {}).result, {}, false};
}

void ModelApp::begin_stroke(bool erase) {
    if (busy() || !doc.has_scan() || (!erase && active_label == 0)) return;
    const model::Label* l = doc.label(active_label);
    doc.begin_change(erase ? "erase paint" : std::format("paint {}", l ? l->name : "?"), model::Author::user);
    stroke_ = Stroke{erase, 0};
}

void ModelApp::dab(const Vec3f& origin, const Vec3f& direction) {
    if (!stroke_) return;
    const auto hit = doc.bvh().raycast(origin, direction.normalized());
    if (!hit) return;
    json p = {{"point", model::vec_to_json(hit->point.cast<double>())}, {"radius", brush_radius}};
    if (stroke_->erase) p["erase"] = true;
    if (active_label != 0) p["label"] = active_label;
    if (doc.apply("paint", p).ok) ++stroke_->dabs;
}

void ModelApp::end_stroke() {
    if (!stroke_) return;
    doc.end_change();
    // A stroke that painted nothing leaves no undo step behind it.
    if (stroke_->dabs == 0) doc.undo();
    stroke_.reset();
}

std::optional<std::string> ModelApp::label_at(const Vec3f& origin, const Vec3f& direction) const {
    if (busy() || !doc.has_scan()) return std::nullopt;
    const auto hit = doc.bvh().raycast(origin, direction.normalized());
    if (!hit) return std::nullopt;
    const int id = (*doc.state().region)[hit->triangle];
    const int painted = (*doc.state().paint)[hit->triangle];
    const model::Label* l = doc.label(id ? id : painted);
    return l ? std::optional(l->name) : std::nullopt;
}

void ModelApp::frame(render::ViewCamera& camera, const std::string& label) const {
    if (!doc.has_scan()) return;
    Eigen::AlignedBox3f box;
    const auto& mesh = doc.mesh();
    if (const model::Label* l = label.empty() ? nullptr : doc.find_label(label)) {
        const auto& region = *doc.state().region;
        for (std::size_t t = 0; t < region.size(); ++t)
            if (region[t] == l->id)
                for (const auto v : mesh.triangles[t]) box.extend(mesh.vertices[v]);
    }
    if (box.isEmpty())
        for (const auto& v : mesh.vertices) box.extend(v);
    camera.target = box.center();
    camera.distance = std::max(20.0f, box.diagonal().norm() / (2 * std::tan(camera.fov_y / 2)) * 1.15f);
}

bool ModelApp::look_from(render::ViewCamera& camera, std::string_view preset) const {
    // The datum's axes if there is one (the part's own), else the scan's.
    Mat3 R = Mat3::Identity();
    if (!doc.state().datums.empty()) R = doc.state().datums.front().frame.linear();
    Vec3 forward, up;
    if (preset == "top") forward = -R.col(2), up = R.col(1);
    else if (preset == "bottom") forward = R.col(2), up = R.col(1);
    else if (preset == "front") forward = R.col(1), up = R.col(2);
    else if (preset == "back") forward = -R.col(1), up = R.col(2);
    else if (preset == "right") forward = -R.col(0), up = R.col(2);
    else if (preset == "left") forward = R.col(0), up = R.col(2);
    else if (preset == "iso") forward = (-R.col(0) + R.col(1) - R.col(2)).normalized(), up = R.col(2);
    else if (preset == "scanned") {
        // From the side the scanner saw: against the scan's mean (area-weighted) normal.
        if (!doc.has_scan()) return false;
        Vec3 n = Vec3::Zero();
        const auto& topo = doc.topology();
        for (std::uint32_t t = 0; t < topo.triangle_count(); ++t) n += topo.area(t) * topo.normal(t).cast<double>();
        if (n.norm() < 1e-9) return false;
        forward = -n.normalized();
        up = any_up(forward);
    } else return false;
    // Camera axes in the world: x right, y down, z forward.
    const Vec3 down = -(up - up.dot(forward) * forward).normalized();
    const Vec3 right = down.cross(forward);
    Mat3 C;
    C.col(0) = right;
    C.col(1) = down;
    C.col(2) = forward;
    camera.orientation = render::Quatf(C.cast<float>()).normalized();
    return true;
}

render::Rgba8 ModelApp::label_color(int id) const {
    const model::Label* l = doc.label(id);
    if (!l) return {200, 202, 208, 255};
    if (l->role == model::Role::ignore) return {95, 95, 100, 255};
    const float hue = std::fmod(static_cast<float>(id) * 0.618034f, 1.0f);
    const float sat = l->role == model::Role::face ? 0.55f : 0.3f;
    return hsv(hue, sat, l->role == model::Role::hole ? 0.7f : 0.92f);
}

std::optional<RenderUpdate> ModelApp::take_render_update() {
    if (busy()) return std::nullopt;
    const double tol = settings.tolerance_mm, range = settings.deviation_range_mm;
    const bool want_model = display == Display::model && doc.built() && doc.built()->result.ok;
    const bool scan_changed = doc.has_scan() && shown_scan_ != &doc.mesh();
    if (!scan_changed && doc.revision() == shown_revision_ && display == shown_display_ && show_edges == shown_edges_ && tol == shown_tolerance_ &&
        range == shown_range_ && want_model == shown_model_geometry_)
        return std::nullopt;
    RenderUpdate u;
    if (!doc.has_scan()) {
        u.geometry.emplace();
        shown_scan_ = nullptr;
    } else if (want_model) {
        const auto& t = doc.built()->tessellation;
        if (!shown_model_geometry_ || doc.revision() != shown_revision_ || scan_changed) {
            std::vector<render::MeshVertex> v(t.mesh.vertices.size());
            for (std::size_t i = 0; i < v.size(); ++i) {
                const auto& p = t.mesh.vertices[i];
                const Vec3f n = i < t.mesh.normals.size() ? t.mesh.normals[i] : Vec3f::Zero();
                v[i] = {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()};
            }
            std::vector<std::uint32_t> idx;
            for (const auto& tri : t.mesh.triangles) idx.insert(idx.end(), tri.begin(), tri.end());
            u.geometry.emplace(std::move(v), std::move(idx));
        }
        // Each face in its label's colour.
        u.colors.assign(t.mesh.vertices.size(), {200, 202, 208, 255});
        const auto& names = doc.built()->result.face_names;
        for (std::size_t tr = 0; tr < t.mesh.triangles.size(); ++tr) {
            const int f = t.triangle_face[tr];
            const model::Label* l = f >= 0 && static_cast<std::size_t>(f) < names.size() ? doc.find_label(names[static_cast<std::size_t>(f)]) : nullptr;
            const render::Rgba8 c = l ? label_color(l->id) : render::Rgba8{185, 188, 196, 255};
            for (const auto vtx : t.mesh.triangles[tr]) u.colors[vtx] = c;
        }
        shown_model_geometry_ = true;
    } else {
        const auto& m = doc.mesh();
        if (scan_changed || shown_model_geometry_) {
            std::vector<render::MeshVertex> v(m.vertices.size());
            for (std::size_t i = 0; i < v.size(); ++i) {
                const auto& p = m.vertices[i];
                const Vec3f n = i < m.normals.size() ? m.normals[i] : Vec3f::Zero();
                v[i] = {p.x(), p.y(), p.z(), n.x(), n.y(), n.z()};
            }
            std::vector<std::uint32_t> idx;
            idx.reserve(m.triangles.size() * 3);
            for (const auto& tri : m.triangles) idx.insert(idx.end(), tri.begin(), tri.end());
            u.geometry.emplace(std::move(v), std::move(idx));
            shown_scan_ = &m;
        }
        u.colors.assign(m.vertices.size(), {200, 202, 208, 255});
        if (display == Display::photos) {
            // Where no photo sees the part: a darker grey.
            for (std::size_t v = 0; v < u.colors.size(); ++v)
                u.colors[v] = v < photo_colours_.size() && photo_colours_[v][3]
                                  ? render::Rgba8{photo_colours_[v][0], photo_colours_[v][1], photo_colours_[v][2], 255}
                                  : render::Rgba8{120, 122, 128, 255};
        } else if (display == Display::deviation && doc.built() && doc.built()->result.ok) {
            const auto& d = doc.built()->deviation.distance;
            for (std::size_t v = 0; v < u.colors.size() && v < d.size(); ++v) {
                const auto c = fit::deviation_color(d[v], static_cast<float>(tol), static_cast<float>(range));
                u.colors[v] = {c[0], c[1], c[2], c[3]};
            }
        } else {
            // Grown regions in their label's colour; painted seeds brighter.
            const auto& region = *doc.state().region;
            const auto& paint = *doc.state().paint;
            for (std::size_t t = 0; t < m.triangles.size(); ++t) {
                const int id = region[t] ? region[t] : paint[t];
                if (id == 0) continue;
                render::Rgba8 c = label_color(id);
                if (paint[t] != 0) c = {static_cast<std::uint8_t>(std::min(255, c.r + 35)), static_cast<std::uint8_t>(std::min(255, c.g + 35)),
                                        static_cast<std::uint8_t>(std::min(255, c.b + 35)), 255};
                for (const auto vtx : m.triangles[t]) u.colors[vtx] = c;
            }
        }
        shown_model_geometry_ = false;
    }
    if (show_edges && doc.built() && doc.built()->result.ok)
        for (const auto& e : doc.built()->tessellation.edges) {
            const render::Rgba8 c{20, 22, 28, 255};
            u.lines.push_back({e[0].x(), e[0].y(), e[0].z(), c});
            u.lines.push_back({e[1].x(), e[1].y(), e[1].z(), c});
        }
    shown_revision_ = doc.revision();
    shown_display_ = display;
    shown_edges_ = show_edges;
    shown_tolerance_ = tol;
    shown_range_ = range;
    return u;
}

void ModelApp::draw_ui(render::ViewCamera& camera) {
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(480, io.DisplaySize.y - 60), ImGuiCond_FirstUseEver);
    ImGui::Begin("Model");

    // ---- File ----
    ImGui::BeginDisabled(busy());
    if (ImGui::Button("Open...")) {
        if (const auto p = choose_open_file({"emodel", "estr", "stl", "ply"})) open(*p);
    }
    ImGui::SameLine();
    if (ImGui::Button("Demo part")) run("open_demo", {});
    ImGui::SameLine();
    ImGui::BeginDisabled(!doc.has_scan());
    if (ImGui::Button("Save")) {
        if (!doc.path().empty()) run("save", {});
        else if (const auto p = choose_save_file("part.emodel", "emodel")) run("save", {{"path", p->string()}});
    }
    ImGui::SameLine();
    if (ImGui::Button("Save as...")) {
        if (const auto p = choose_save_file("part.emodel", "emodel")) run("save", {{"path", p->string()}});
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!(doc.built() && doc.built()->result.ok));
    if (ImGui::Button("Export STEP...")) {
        if (const auto p = choose_save_file("part.step", "step")) start("export_step", {{"path", p->string()}});
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    if (!settings.recent.empty() && ImGui::BeginCombo("Recent", "open a recent file")) {
        for (const auto& r : settings.recent)
            if (ImGui::Selectable(r.c_str())) open(r);
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    if (busy()) {
        // The document is being worked on in the background: nothing reads it until that is done.
        ImGui::TextColored(ImVec4(1, 0.85f, 0.3f, 1), "%s", busy_text_.c_str());
        ImGui::End();
        return;
    }
    if (!doc.has_scan()) {
        ImGui::TextWrapped("Open a scan (.estr from the Einstar app, or an STL/PLY mesh) or a saved model, or try the demo part.");
        ImGui::End();
        return;
    }
    const json summary = doc.apply("summary", {}).result;
    ImGui::TextDisabled("%s  %zu triangles, voxel %.2f mm", summary["scan"]["kind"].get<std::string>().c_str(),
                        summary["scan"]["triangles"].get<std::size_t>(), doc.voxel_mm());
    if (const auto source = summary["scan"]["source"].get<std::string>(); source == "changed") {
        ImGui::TextColored(ImVec4(1, 0.85f, 0.3f, 1), "The scan's file has changed since this mesh was made from it.");
        if (ImGui::Button("Process it again (labels carried over)")) start("reprocess", json::object());
    } else if (source == "missing") {
        ImGui::TextDisabled("The scan's file is gone: the model keeps its own copy of the mesh.");
    }

    // ---- View ----
    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
        int d = static_cast<int>(display);
        // ##view keeps the "Labels" caption but a distinct ID from the Labels section header below.
        ImGui::RadioButton("Labels##view", &d, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Deviation", &d, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Model", &d, 2);
        const bool registered = std::ranges::any_of(doc.state().photos, [](const model::Photo& ph) { return ph.camera.has_value(); });
        ImGui::SameLine();
        ImGui::BeginDisabled(!registered);
        ImGui::RadioButton("Photos", &d, 3);
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("The scan coloured from its registered photos (Photo window: Match to scan).");
        display = static_cast<Display>(d);
        ImGui::SameLine();
        ImGui::Checkbox("Edges", &show_edges);
        for (const char* p : {"top", "front", "right", "iso"}) {
            if (ImGui::SmallButton(p)) look_from(camera, p);
            ImGui::SameLine();
        }
        if (ImGui::SmallButton("frame all")) frame(camera);
        if (display == Display::deviation) {
            float tol = static_cast<float>(settings.tolerance_mm), range = static_cast<float>(settings.deviation_range_mm);
            if (ImGui::SliderFloat("Tolerance mm", &tol, 0.01f, 1.0f, "%.3f")) settings.tolerance_mm = tol;
            if (ImGui::SliderFloat("Colour range mm", &range, 0.05f, 3.0f, "%.2f")) settings.deviation_range_mm = range;
            if (ImGui::IsItemDeactivatedAfterEdit()) (void)model::save_settings(settings);
            if (!doc.built()) ImGui::TextDisabled("Build the model to see the deviation.");
        }
    }

    // ---- Labels ----
    if (ImGui::CollapsingHeader("Labels", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Detect faces")) start("detect", {});
        ImGui::SameLine();
        if (ImGui::Button("New label")) {
            const auto o = run("label.create", {});
            if (o.ok) active_label = o.result["id"];
        }
        ImGui::SameLine();
        if (ImGui::Button("Grow")) start("grow", {});
        ImGui::SliderFloat("Brush mm", &brush_radius, 0.3f, 15.0f, "%.1f");
        ImGui::TextDisabled("Shift-drag paints the selected label, Option-drag erases.");
        if (ImGui::BeginTable("labels", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp,
                              ImVec2(0, 220))) {
            for (const auto& l : summary["labels"]) {
                const int id = l["id"];
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::ColorButton(std::format("##c{}", id).c_str(), im(label_color(id)), ImGuiColorEditFlags_NoTooltip, ImVec2(14, 14));
                ImGui::SameLine();
                if (ImGui::Selectable(std::format("{}##l{}", l["name"].get<std::string>(), id).c_str(), active_label == id,
                                      ImGuiSelectableFlags_SpanAllColumns))
                    active_label = id;
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%s", l["role"].get<std::string>().c_str());
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%s", l.contains("fit") ? l["fit"]["kind"].get<std::string>().c_str() : l.contains("given") ? "given" : "-");
                ImGui::TableNextColumn();
                ImGui::TextDisabled("%d", l.value("triangles", 0));
            }
            ImGui::EndTable();
        }
        if (const model::Label* l = doc.label(active_label)) {
            if (rename_for_ != l->id) {
                std::snprintf(rename_, sizeof(rename_), "%s", l->name.c_str());
                rename_for_ = l->id;
            }
            if (ImGui::InputText("Name", rename_, sizeof(rename_), ImGuiInputTextFlags_EnterReturnsTrue))
                run("label.update", {{"label", l->id}, {"name", std::string(rename_)}});
            int role = static_cast<int>(l->role);
            if (ImGui::Combo("Role", &role, "face\0hole\0fillet\0ignore\0"))
                run("label.update", {{"label", l->id}, {"role", model::role_name(static_cast<model::Role>(role))}});
            if (ImGui::Button("Find holes")) start("find_holes", {{"label", l->id}});
            ImGui::SameLine();
            if (ImGui::Button("As fillet")) run("fillet.add", {{"label", l->id}});
            ImGui::SameLine();
            if (ImGui::Button("Freeform")) start("freeform", {{"label", l->id}});
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("A smooth freeform face over the label's region (or its paint, taken out to sharp edges).");
            ImGui::SameLine();
            if (ImGui::Button("Frame")) frame(camera, l->name);
            ImGui::SameLine();
            if (ImGui::Button("Delete label")) {
                run("label.delete", {{"label", l->id}});
                active_label = 0;
            }
        }
    }

    // ---- Holes and fillets ----
    if (ImGui::CollapsingHeader("Holes and fillets")) {
        for (const auto& h : summary["holes"]) {
            ImGui::PushID(h["id"].get<int>());
            ImGui::Text("%s", h["name"].get<std::string>().c_str());
            ImGui::SameLine(110);
            double dia = h["diameter"];
            ImGui::SetNextItemWidth(90);
            if (ImGui::InputDouble("dia", &dia, 0, 0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) run("hole.update", {{"hole", h["id"]}, {"diameter", dia}});
            ImGui::SameLine();
            ImGui::TextDisabled("scan %.3f (%s)", h["measured_diameter"].get<double>(), h["measured_from"].get<std::string>().c_str());
            std::string forms;
            if (h.contains("counterbore_diameter"))
                forms += std::format("  counterbore {:.2f} x {:.2f}", h["counterbore_diameter"].get<double>(), h.value("counterbore_depth", 0.0));
            if (h.contains("countersink_diameter"))
                forms += std::format("  countersink {:.2f} at {:.0f} deg", h["countersink_diameter"].get<double>(), h.value("countersink_angle_deg", 90.0));
            if (h.contains("point_angle_deg")) forms += std::format("  drill point {:.0f} deg", h["point_angle_deg"].get<double>());
            if (!forms.empty()) ImGui::TextDisabled("%s", forms.c_str());
            bool through = h["through"];
            if (ImGui::Checkbox("through", &through) && through) run("hole.update", {{"hole", h["id"]}, {"through", true}});
            if (!through) {
                ImGui::SameLine();
                double depth = h.value("depth", 0.0);
                ImGui::SetNextItemWidth(90);
                if (ImGui::InputDouble("depth", &depth, 0, 0, "%.2f", ImGuiInputTextFlags_EnterReturnsTrue)) run("hole.update", {{"hole", h["id"]}, {"depth", depth}});
            } else {
                ImGui::SameLine();
                ImGui::TextDisabled("wall seen to %.1f mm", h["wall_seen_to_depth"].get<double>());
            }
            ImGui::PopID();
        }
        for (const auto& f : summary["fillets"]) {
            ImGui::PushID(f["id"].get<int>() + 100000);
            ImGui::Text("%s", f["name"].get<std::string>().c_str());
            double r = f["radius"];
            ImGui::SetNextItemWidth(90);
            if (ImGui::InputDouble("R", &r, 0, 0, "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) run("fillet.update", {{"fillet", f["id"]}, {"radius", r}});
            ImGui::SameLine();
            ImGui::TextDisabled("scan %.3f", f["measured_radius"].get<double>());
            ImGui::PopID();
        }
    }

    // ---- Datums and constraints ----
    const auto names = label_names(doc);
    std::vector<std::string> datums;
    for (const auto& x : doc.state().datums) datums.push_back(x.name);
    if (ImGui::CollapsingHeader("Datums and constraints", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SetNextItemWidth(120);
        combo("z from##dz", datum_form_.z, names);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        combo("x from##dx", datum_form_.x, names);
        ImGui::SameLine();
        if (ImGui::Button("New datum") && !names.empty())
            run("datum.create", {{"z", names[static_cast<std::size_t>(datum_form_.z)]}, {"x", names[static_cast<std::size_t>(datum_form_.x)]}});
        if (!datums.empty()) {
            if (ImGui::Button("Square to datum")) run("square", {{"datum", datums.front()}});
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Aligns every plane face to the nearest axis of the first datum.");
            ImGui::SetNextItemWidth(80);
            combo("##pd", plane_form_.datum, datums);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(50);
            ImGui::Combo("##pa", &plane_form_.axis, "x\0y\0z\0");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80);
            ImGui::InputDouble("##po", &plane_form_.offset, 0, 0, "%.2f");
            ImGui::SameLine();
            ImGui::Checkbox("faces -##pf", &plane_form_.facing_minus);
            ImGui::SameLine();
            if (ImGui::Button("Add unseen face"))
                run("face.add_plane", {{"datum", datums[static_cast<std::size_t>(plane_form_.datum)]},
                                       {"axis", std::string(1, "xyz"[plane_form_.axis])},
                                       {"offset", plane_form_.offset},
                                       {"facing", plane_form_.facing_minus ? "-" : "+"}});
        }
        ImGui::Separator();
        std::vector<std::string> types(kConstraintTypes.begin(), kConstraintTypes.end());
        ImGui::SetNextItemWidth(120);
        combo("##ct", form_.type, types);
        const std::string type = types[static_cast<std::size_t>(form_.type)];
        const bool pair = type == "parallel" || type == "perpendicular" || type == "angle" || type == "coplanar" || type == "coaxial" ||
                          type == "distance" || type == "axis_distance" || type == "tangent" || type == "symmetric" || type == "equal_radius";
        const bool on_datum = type == "aligned" || type == "offset" || type == "symmetric";
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110);
        combo(pair ? "##ca" : "##cl", form_.a, names);
        if (pair) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(110);
            combo("##cb", form_.b, names);
        }
        if (on_datum && !datums.empty()) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(70);
            combo("##cd", form_.datum, datums);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(40);
            ImGui::Combo("##cx", &form_.axis, "x\0y\0z\0");
        }
        if (type == "angle" || type == "radius" || type == "diameter" || type == "distance" || type == "offset" || type == "axis_distance") {
            ImGui::SetNextItemWidth(100);
            ImGui::InputDouble(type == "angle" ? "degrees" : "mm", &form_.value, 0, 0, "%.4f");
            ImGui::SameLine();
        }
        if (ImGui::Button("Add constraint") && !names.empty()) {
            json p = {{"type", type}};
            const auto name = [&](int i) { return names[static_cast<std::size_t>(std::clamp(i, 0, static_cast<int>(names.size()) - 1))]; };
            if (pair) p["a"] = name(form_.a), p["b"] = name(form_.b);
            else p["label"] = name(form_.a);
            if (on_datum && !datums.empty()) p["datum"] = datums[static_cast<std::size_t>(form_.datum)], p["axis"] = std::string(1, "xyz"[form_.axis]);
            if (type == "angle") p["degrees"] = form_.value;
            else if (type != "tangent" && type != "symmetric" && type != "equal_radius" && type != "parallel" && type != "perpendicular" &&
                     type != "coplanar" && type != "coaxial" && type != "aligned")
                p["value"] = form_.value;
            run("constraint.add", p);
        }
        for (const auto& c : summary["constraints"]) {
            const auto& spec = c["constraint"];
            std::string text = spec["type"].get<std::string>();
            for (const char* key : {"label", "hole", "a", "b", "datum", "axis", "value", "degrees"})
                if (spec.contains(key)) text += " " + (spec[key].is_string() ? spec[key].get<std::string>() : spec[key].dump());
            const std::string status = c.contains("last_solve") ? c["last_solve"]["status"].get<std::string>() : "not solved";
            ImGui::PushID(c["id"].get<int>());
            if (ImGui::SmallButton("x")) run("constraint.remove", {{"constraint", c["id"]}});
            ImGui::SameLine();
            ImGui::TextColored(status_color(status), "%s", text.c_str());
            if (ImGui::IsItemHovered() && c.contains("last_solve"))
                ImGui::SetTooltip("%s\nmoves the fit by %.4f mm, rms %+.4f mm", status.c_str(), c["last_solve"]["moves_scan_fit_mm"].get<double>(),
                                  c["last_solve"]["rms_change_mm"].get<double>());
            ImGui::PopID();
        }
    }

    // ---- Solve and build ----
    if (ImGui::CollapsingHeader("Solve and build", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Solve")) start("solve", {});
        ImGui::SameLine();
        if (ImGui::Button("Build")) start("build", {{"tolerance_mm", settings.tolerance_mm}});
        if (summary["model"].is_object()) {
            const auto& m = summary["model"];
            ImGui::Text("%s%s, %.1f mm3, %d faces, scan coverage %.0f%%", m["ok"].get<bool>() ? (m["closed"].get<bool>() ? "solid" : "open") : "failed",
                        m["current"].get<bool>() ? "" : " (out of date)", m["volume_mm3"].get<double>(), m["faces"].get<int>(),
                        100 * m["scan_coverage"].get<double>());
            for (const auto& line : m["log"]) ImGui::TextWrapped("%s", line.get<std::string>().c_str());
        }
        if (doc.built() && doc.built()->result.ok) {
            const auto& r = doc.built()->report;
            ImGui::Text("Deviation: rms %.3f, p95 %.3f mm; %.1f%% within %.2f mm", r.overall.rms, r.overall.p95, 100 * r.overall.within_tolerance,
                        settings.tolerance_mm);
            int shown = 0;
            for (const auto& h : r.hot_spots) {
                if (h.edge_band || shown >= 8) continue;
                ImGui::PushID(shown++);
                if (ImGui::SmallButton("go")) {
                    camera.target = h.centroid.cast<float>();
                    camera.distance = std::max(30.0f, static_cast<float>(4 * std::sqrt(h.area_mm2)));
                    display = Display::deviation;
                }
                ImGui::SameLine();
                ImGui::Text("%+.3f mm over %.1f mm2", h.mean_mm, h.area_mm2);
                ImGui::PopID();
            }
        }
    }

    // ---- History (user and agent) ----
    if (ImGui::CollapsingHeader("History")) {
        if (ImGui::Button("Undo")) run("undo", {});
        ImGui::SameLine();
        if (ImGui::Button("Redo")) run("redo", {});
        const json h = doc.history();
        for (auto it = h["undo"].rbegin(); it != h["undo"].rend(); ++it) {
            const std::string author = (*it)["author"];
            ImGui::TextColored(author == "agent" ? ImVec4(0.55f, 0.75f, 1, 1) : ImVec4(0.8f, 0.8f, 0.8f, 1), "%s  %s", author == "agent" ? "agent" : "     ",
                               (*it)["description"].get<std::string>().c_str());
        }
    }
    ImGui::End();
}

}  // namespace einstar::modelapp
