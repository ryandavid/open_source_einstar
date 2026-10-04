#pragma once

// CAD solids from fitted surfaces (OpenCASCADE behind this interface), and STEP export.
//
// The part's main faces (planes and large curved faces) are extended past the part and split space into
// cells (BOPAlgo_MakerVolume). Each cell is material or air by a vote of the scan: the scan points
// supporting a face lie on the pieces of it that bound material, and the scan's normals say which side of
// each piece the material is on. The material cells are fused and coplanar pieces merged. Blocks (regions
// bounded by their own planes, which stay out of the arrangement) are added or cut. Holes are then
// cut and fillets rolled on the edges between their two faces, rather than putting those surfaces into
// the arrangement, where near-tangent fillets make the booleans fragile.

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "einstar/core/error.hpp"
#include "einstar/fit/surface.hpp"
#include "einstar/recon/mesh.hpp"

namespace einstar::brep {

struct FaceInput {
    std::string name;            // carried into the STEP file
    fit::Surface surface;
    std::vector<Vec3> points;    // scan support (may be empty: a face the scan did not see, e.g. a bottom)
    std::vector<Vec3> normals;   // scan normals of the points (pointing out of the material)
    bool arrangement = true;     // false: it only bounds blocks (it does not split space into cells)
};

// A region bounded by planes, added to the solid or cut from it after the cells are joined: a feature the
// scan barely saw (a clip, a boss) or one that is hollow (a pocket), which the cells' vote cannot make.
struct BlockFace {
    int face = 0;          // index into faces (a plane)
    bool outside = false;  // the region lies past the face (on its outward side), not on its material side
};
struct BlockInput {
    std::string name;
    std::vector<BlockFace> faces;
    bool cut = false;  // removed from the solid (a pocket), else added
};

struct HoleInput {
    std::string name;
    Vec3 entry = Vec3::Zero();   // centre of the opening, on the entry face
    Vec3 axis = -Vec3::UnitZ();  // unit, into the material
    double diameter = 5;
    std::optional<double> depth; // blind hole (to the shoulder if it has a point); nullopt: through
    std::optional<double> counterbore_diameter, counterbore_depth;
    std::optional<double> countersink_diameter, countersink_angle_deg;  // included angle
    std::optional<double> point_angle_deg;                               // a drill point at a blind hole's bottom
};

struct FilletInput {
    std::string name;
    int face_a = 0, face_b = 0;  // indices into faces: the fillet rolls along every edge between them
    double radius = 1;
};

struct BuildInput {
    std::vector<FaceInput> faces;
    std::vector<HoleInput> holes;
    std::vector<FilletInput> fillets;
    std::vector<BlockInput> blocks;
    double margin_mm = 10;           // faces extend this far past the part's bounding box
    std::size_t max_vote_points = 400;  // per face
};

struct Shape;  // OpenCASCADE shape (opaque)

struct BuildResult {
    std::shared_ptr<const Shape> shape;
    bool ok = false;         // a valid shape was made
    bool closed = false;     // one or more closed solids (otherwise an open shell)
    double volume = 0;       // mm^3 (closed only)
    int solids = 0, faces = 0, edges = 0;
    // Share of the scan's support on the solid's boundary; low when the faces do not enclose the part (a
    // face missing where the scan is open), and then the solid is only part of it.
    double scan_coverage = 0;
    std::vector<std::string> log;  // what was done and what failed (e.g. a fillet that could not be made)
    // Name of each face of the shape, in TopExp order (faces matched to the input surface they lie on).
    std::vector<std::string> face_names;
};

[[nodiscard]] BuildResult build(const BuildInput& input);

// AP242, millimetres, faces named after their features.
Result<void> write_step(const BuildResult& result, const std::filesystem::path& path, const std::string& part_name = "part");

// Triangles of the shape for display: a feature name per triangle (index into face_names, -1 unnamed),
// and the shape's edges as line segments.
struct Tessellation {
    recon::TriangleMesh mesh;
    std::vector<int> triangle_face;
    std::vector<std::array<Vec3f, 2>> edges;
};
[[nodiscard]] Tessellation tessellate(const BuildResult& result, double deflection_mm = 0.05);

// What a STEP file holds (to check an export by reading it back).
struct StepSummary {
    bool ok = false;
    std::string error;
    int solids = 0, shells = 0, faces = 0, edges = 0;
    bool valid = false;  // BRepCheck
    double volume = 0;
    std::map<std::string, int> surface_kinds;  // "plane", "cylinder", "cone", "sphere", "torus", "other"
    std::vector<double> cylinder_radii;        // sorted, deduplicated to 1e-6
    std::vector<std::string> face_names;       // named faces found in the file
};
[[nodiscard]] StepSummary read_step(const std::filesystem::path& path);

}  // namespace einstar::brep
