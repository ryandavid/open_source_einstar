#include "einstar/model/document.hpp"

#include <algorithm>

namespace einstar::model {

std::string_view role_name(Role r) {
    switch (r) {
        case Role::face: return "face";
        case Role::hole: return "hole";
        case Role::fillet: return "fillet";
        case Role::ignore: return "ignore";
    }
    return "?";
}

std::optional<Role> role_from_name(std::string_view s) {
    for (const Role r : {Role::face, Role::hole, Role::fillet, Role::ignore})
        if (role_name(r) == s) return r;
    return std::nullopt;
}

std::string_view author_name(Author a) {
    switch (a) {
        case Author::user: return "user";
        case Author::agent: return "agent";
        case Author::script: return "script";
    }
    return "?";
}

Document::Document() = default;
Document::~Document() = default;

std::string ScanSource::status() const {
    if (path.empty()) return "none";
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) return "missing";
    const auto t = std::filesystem::last_write_time(path, ec);
    const auto now_mtime = ec ? 0 : static_cast<std::int64_t>(t.time_since_epoch().count());
    return std::filesystem::file_size(path, ec) != size || now_mtime != mtime ? "changed" : "unchanged";
}

void Document::set_scan(recon::TriangleMesh mesh, ScanSource source) {
    adopt_mesh(std::move(mesh));
    source_ = std::move(source);
    state_ = State{};
    state_.paint = std::make_shared<const std::vector<std::uint16_t>>(mesh_->triangles.size(), 0);
    state_.region = std::make_shared<const std::vector<std::uint16_t>>(mesh_->triangles.size(), 0);
    undo_.clear();
    redo_.clear();
    change_depth_ = 0;
    built_.reset();
    ++revision_;
}

void Document::adopt_mesh(recon::TriangleMesh mesh) {
    if (mesh.normals.size() != mesh.vertices.size()) mesh.compute_normals();
    bvh_.reset();
    mesh_ = std::make_unique<recon::TriangleMesh>(std::move(mesh));
    topo_ = std::make_unique<fit::MeshTopology>(*mesh_);
    bvh_ = std::make_unique<fit::TriangleBvh>(*mesh_);
    // The scan's resolution: the median edge length is about the processing voxel.
    std::vector<float> edges;
    for (std::size_t t = 0; t < mesh_->triangles.size(); t += std::max<std::size_t>(1, mesh_->triangles.size() / 2000)) {
        const auto& tri = mesh_->triangles[t];
        edges.push_back((mesh_->vertices[tri[0]] - mesh_->vertices[tri[1]]).norm());
    }
    if (!edges.empty()) {
        std::ranges::nth_element(edges, edges.begin() + static_cast<std::ptrdiff_t>(edges.size() / 2));
        voxel_mm_ = std::clamp(static_cast<double>(edges[edges.size() / 2]), 0.05, 5.0);
    }
}

void Document::scale_scan(double k) {
    if (!mesh_) return;
    for (auto& v : mesh_->vertices) v *= static_cast<float>(k);
    topo_ = std::make_unique<fit::MeshTopology>(*mesh_);
    bvh_ = std::make_unique<fit::TriangleBvh>(*mesh_);
    voxel_mm_ *= k;
    source_.scale *= k;
    const auto sized = [&](std::optional<double>& v) {
        if (v) *v *= k;
    };
    for (auto& l : state_.labels) {
        if (l.fit) l.fit = fit::scaled(*l.fit, k);
        if (l.given) l.given = fit::scaled(*l.given, k);
        l.sigma *= k;
        l.rms *= k;
    }
    for (auto& [id, s] : state_.solved) s = fit::scaled(s, k);
    for (auto& h : state_.holes) {
        // Measured from the scan: scaled. Set by the user (a caliper reading): kept.
        h.center *= k;
        h.measured_center *= k;
        h.measured_diameter *= k;
        h.seen_depth *= k;
        sized(h.depth);
        sized(h.counterbore_diameter);
        sized(h.counterbore_depth);
        sized(h.countersink_diameter);
    }
    for (auto& f : state_.fillets) f.measured_radius *= k;
    for (auto& x : state_.datums) x.frame.translation() *= k;
    // Offsets in constraints are measurements: kept.
    undo_.clear();
    redo_.clear();
    change_depth_ = 0;
    built_.reset();
    ++revision_;
}

const Label* Document::label(int id) const {
    const auto it = std::ranges::find(state_.labels, id, &Label::id);
    return it == state_.labels.end() ? nullptr : &*it;
}

const Label* Document::find_label(const json& ref) const {
    if (ref.is_number_integer()) return label(ref.get<int>());
    if (ref.is_string()) {
        const auto name = ref.get<std::string>();
        const auto it = std::ranges::find(state_.labels, name, &Label::name);
        return it == state_.labels.end() ? nullptr : &*it;
    }
    return nullptr;
}

void Document::snapshot(std::string description, Author author) {
    undo_.push_back({std::move(description), author, state_, revision_});
    redo_.clear();
    constexpr std::size_t kMaxUndo = 200;
    if (undo_.size() > kMaxUndo) undo_.erase(undo_.begin());
}

void Document::begin_change(std::string description, Author author) {
    if (change_depth_++ == 0) snapshot(std::move(description), author);
}

void Document::end_change() {
    if (change_depth_ > 0) --change_depth_;
}

bool Document::undo() {
    if (undo_.empty()) return false;
    Step step = std::move(undo_.back());
    undo_.pop_back();
    redo_.push_back({step.description, step.author, state_, revision_});
    state_ = std::move(step.before);
    change_depth_ = 0;
    ++revision_;
    return true;
}

bool Document::redo() {
    if (redo_.empty()) return false;
    Step step = std::move(redo_.back());
    redo_.pop_back();
    undo_.push_back({step.description, step.author, state_, revision_});
    state_ = std::move(step.before);
    ++revision_;
    return true;
}

json Document::history() const {
    json out = {{"undo", json::array()}, {"redo", json::array()}, {"revision", revision_}};
    for (const auto& s : undo_) out["undo"].push_back({{"description", s.description}, {"author", author_name(s.author)}});
    for (const auto& s : redo_) out["redo"].push_back({{"description", s.description}, {"author", author_name(s.author)}});
    return out;
}

}  // namespace einstar::model
