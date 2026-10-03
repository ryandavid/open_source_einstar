#pragma once

// Synthetic machined parts with known geometry, meshed the way a scan is (surface nets on a signed
// distance volume), for tests and the modelling app's demo part. Each triangle carries the id of the
// analytic face it came from, so recovered surfaces can be checked against the truth.
//
// A part is a union of boxes (each with an optional fillet radius on all of its edges) minus round holes.
// Like a real scan, the generator can leave out the bottom face (the part stood on a table) and the
// inside of holes deeper than the scanner can see.

#include <cstdint>
#include <string>
#include <vector>

#include "einstar/fit/surface.hpp"
#include "einstar/recon/mesh.hpp"

namespace einstar::fit {

struct PartBox {
    Vec3 min = Vec3::Zero(), max = Vec3::Ones();
    double fillet = 0;  // radius on every edge (0: sharp)
};

struct PartHole {
    Vec3 entry = Vec3::Zero();  // centre of the opening, on the entry face
    Vec3 axis = -Vec3::UnitZ(); // unit, pointing into the material
    double diameter = 5;
    double depth = 1e9;         // flat-bottomed blind hole; larger than the material: through
};

struct PartSpec {
    std::vector<PartBox> boxes;
    std::vector<PartHole> holes;

    double voxel_mm = 0.5;
    double noise_mm = 0.03;   // normal noise added to the mesh vertices
    std::uint32_t seed = 1;
    // Surface below this height is not observed (the part stood on a table); -inf: everything is seen.
    double unseen_below_z = -1e9;
    // Hole walls further than this many diameters from an opening are not observed.
    double hole_wall_depth = 1.0;
    bool simplify = false;    // quadric simplification as the process step does
};

struct TruthFace {
    int id = 0;
    std::string name;  // e.g. "box0 +z", "box0 fillet x+y-", "hole1 wall"
    Surface surface;
    bool hole = false;  // wall or bottom of a hole (material outside the cylinder)
};

struct SyntheticPart {
    recon::TriangleMesh mesh;
    std::vector<int> triangle_face;  // TruthFace::id per triangle
    std::vector<TruthFace> faces;    // every face of the description (some may have no triangles)

    [[nodiscard]] const TruthFace* face(int id) const;
    [[nodiscard]] std::size_t triangles_of(int id) const;
};

[[nodiscard]] SyntheticPart make_synthetic_part(const PartSpec& spec);

// A box with mounting flanges, the example the app was designed around:
//   body 60 x 40 x 20 mm (z 0..20), 2 mm fillets on its edges;
//   flange plate 100 x 50 x 4 mm (z 0..4), sharp, with four 5.5 mm through holes at (+-42, +-19);
//   an 8 mm blind hole 10 mm deep in the top of the body.
// The bottom (z < 0.3) is unseen, as if the part stood on a table.
[[nodiscard]] PartSpec flanged_box_spec();

}  // namespace einstar::fit
