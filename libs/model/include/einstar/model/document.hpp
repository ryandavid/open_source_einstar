#pragma once

// The modelling document: a scan and everything modelled on it.
//
// Labels are named regions of the scan (painted seeds grown into faces), each with the surface fitted to it
// and a role: a face of the part, the wall of a hole, a fillet, or scan to ignore (a fixture, the table).
// Holes, fillets, datums and constraints refer to labels. Solving fits every face together under the
// constraints; building makes the solid (libs/brep) and measures the scan's deviation from it.
//
// Every change goes through apply(command, params): the app's UI, the agent (MCP, through the app's
// `model.*` methods) and tests drive the same commands, each one undoable and recorded with its author.
// Commands and their parameters are described in the agent's method table (libs/agent specs.cpp); a test
// keeps the two in step.

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "einstar/brep/brep.hpp"
#include "einstar/core/error.hpp"
#include "einstar/fit/bvh.hpp"
#include "einstar/fit/constraints.hpp"
#include "einstar/fit/deviation.hpp"
#include "einstar/fit/mesh_topology.hpp"
#include "einstar/fit/surface.hpp"
#include "einstar/recon/mesh.hpp"

namespace einstar::model {

using json = nlohmann::json;

enum class Role { face, hole, fillet, ignore };
[[nodiscard]] std::string_view role_name(Role r);
[[nodiscard]] std::optional<Role> role_from_name(std::string_view s);

struct Label {
    int id = 0;  // 1..65535 (0 marks unlabelled triangles)
    std::string name;
    Role role = Role::face;
    std::vector<fit::SurfaceKind> kinds;  // allowed surface kinds when growing (empty: any)
    std::optional<fit::Surface> fit;      // fitted to the label's region
    double sigma = 0, rms = 0;
    // A face the scan did not see (e.g. the bottom the part stood on): its surface is given, not fitted.
    std::optional<fit::Surface> given;
};

struct Hole {
    int id = 0;
    std::string name;
    int host = 0;                  // the plane label it opens in
    Vec3 center = Vec3::Zero();    // on the host face (where the last solve put it)
    Vec3 measured_center = Vec3::Zero();  // as found in the scan
    Vec3 axis = -Vec3::UnitZ();    // into the material
    double measured_diameter = 0;  // from the scan
    std::optional<double> diameter;  // set by the user (a caliper reading); overrides the measurement
    std::optional<double> depth;   // blind hole; nullopt: through
    bool wall_seen = false;        // the measurement comes from the wall (else from the opening)
    double seen_depth = 0;         // how deep the scan saw its wall (a through hole's far end, or a blind hole's
                                   // floor, may not have been seen: the depth is then the user's to give)
    int wall_label = 0;            // a label on its wall, if any
    std::vector<std::uint32_t> rim;  // the opening's boundary (scan vertices): its position when no wall is labelled
    // Forms, measured from the scan or set: a counterbore, a countersink (included angle), a drill point at a blind
    // hole's bottom (included angle; the depth is then to the shoulder).
    std::optional<double> counterbore_diameter, counterbore_depth;
    std::optional<double> countersink_diameter, countersink_angle_deg;
    std::optional<double> point_angle_deg;
    [[nodiscard]] double used_diameter() const { return diameter.value_or(measured_diameter); }
};

struct Fillet {
    int id = 0;
    std::string name;
    int label = 0;                // the fillet's own region, if any
    int face_a = 0, face_b = 0;   // the two face labels it joins
    double measured_radius = 0;
    std::optional<double> radius;  // set by the user
    [[nodiscard]] double used_radius() const { return radius.value_or(measured_radius); }
};

struct Datum {
    int id = 0;
    std::string name;
    SE3 frame = SE3::Identity();
};

// A constraint as given (JSON with label and datum ids resolved), e.g.
// {"type": "aligned", "label": 3, "datum": 1, "axis": 2}.
struct ConstraintDef {
    int id = 0;
    json spec;
};

enum class Author { user, agent, script };
[[nodiscard]] std::string_view author_name(Author a);

// Everything that undo restores. The per-triangle arrays are shared between snapshots until changed.
struct State {
    std::shared_ptr<const std::vector<std::uint16_t>> paint;   // seed label per triangle (0: none)
    std::shared_ptr<const std::vector<std::uint16_t>> region;  // grown label per triangle (0: none)
    std::vector<Label> labels;
    std::vector<Hole> holes;
    std::vector<Fillet> fillets;
    std::vector<Datum> datums;
    std::vector<ConstraintDef> constraints;
    std::map<int, fit::Surface> solved;  // label id -> surface after the last solve
    json solve_report;                   // per constraint and label, from the last solve
    int next_id = 1;
};

// Where the scan came from.
struct ScanSource {
    std::filesystem::path path;  // .estr, .stl, .ply (empty for the demo part)
    std::string kind;            // "estr", "stl", "ply", "demo"
    std::uintmax_t size = 0;
    std::int64_t mtime = 0;
    json process;                // process settings used (estr)
    double scale = 1.0;          // applied to the mesh since it was loaded (model.scale)
    // Whether the file has changed (size or modification time) or gone since the mesh was made from it.
    [[nodiscard]] std::string status() const;  // "unchanged", "changed", "missing" or "none" (the demo part)
};

struct Built {
    brep::BuildResult result;
    brep::Tessellation tessellation;
    std::unique_ptr<fit::TriangleBvh> bvh;
    fit::DeviationField deviation;
    fit::DeviationReport report;
    fit::Coverage coverage;
    std::uint64_t revision = 0;  // the document revision it was built from
};

struct Outcome {
    bool ok = true;
    json result;            // on success
    std::string error;      // on failure
    bool refused = false;   // the command cannot be done now (rather than bad parameters)
};

class Document {
public:
    Document();
    ~Document();
    Document(const Document&) = delete;
    Document& operator=(const Document&) = delete;

    // --- the scan ---
    void set_scan(recon::TriangleMesh mesh, ScanSource source);
    // Scales the scan and everything modelled on it about the origin (a calibration correction). The undo history is
    // cleared: earlier steps hold unscaled geometry.
    void scale_scan(double factor);
    [[nodiscard]] bool has_scan() const { return mesh_ != nullptr; }
    [[nodiscard]] const recon::TriangleMesh& mesh() const { return *mesh_; }
    [[nodiscard]] const fit::MeshTopology& topology() const { return *topo_; }
    [[nodiscard]] const fit::TriangleBvh& bvh() const { return *bvh_; }
    [[nodiscard]] const ScanSource& source() const { return source_; }
    [[nodiscard]] double voxel_mm() const { return voxel_mm_; }

    // --- commands ---
    // `command` without the "model." prefix ("label.create", "solve", ...).
    Outcome apply(std::string_view command, const json& params, Author author = Author::user);
    [[nodiscard]] static const std::vector<std::string>& commands();  // every command apply() knows

    // Group the commands until end_change into one undo step (an agent's turn).
    void begin_change(std::string description, Author author);
    void end_change();
    bool undo();
    bool redo();
    [[nodiscard]] std::uint64_t revision() const { return revision_; }
    [[nodiscard]] json history() const;

    [[nodiscard]] const State& state() const { return state_; }
    [[nodiscard]] const Label* label(int id) const;
    [[nodiscard]] const Label* find_label(const json& ref) const;  // by id or name
    [[nodiscard]] const std::optional<Built>& built() const { return built_; }
    [[nodiscard]] bool built_is_current() const { return built_ && built_->revision == revision_; }

    // --- files ---
    Result<void> save(const std::filesystem::path& path) const;
    Result<void> load(const std::filesystem::path& path);
    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

    // Implementation detail of the command handlers (commands.cpp).
    struct Impl;

private:
    friend struct Impl;
    void snapshot(std::string description, Author author);
    void adopt_mesh(recon::TriangleMesh mesh);  // the mesh, its topology, BVH and resolution

    std::unique_ptr<recon::TriangleMesh> mesh_;
    std::unique_ptr<fit::MeshTopology> topo_;
    std::unique_ptr<fit::TriangleBvh> bvh_;
    ScanSource source_;
    double voxel_mm_ = 0.5;
    std::filesystem::path path_;

    State state_;
    struct Step {
        std::string description;
        Author author = Author::user;
        State before;
        std::uint64_t revision = 0;
    };
    std::vector<Step> undo_, redo_;
    int change_depth_ = 0;
    std::uint64_t revision_ = 0;
    std::optional<Built> built_;
};

}  // namespace einstar::model
