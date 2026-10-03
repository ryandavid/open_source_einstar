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
    // A freeform top: z = max.z + dome (1 - u^2)(1 - v^2) over the box's footprint (u, v from -1 to 1), meeting the
    // sides at an angle. 0: flat. (With a dome, fillets are not supported.)
    double dome = 0;
};

struct PartHole {
    Vec3 entry = Vec3::Zero();  // centre of the opening, on the entry face
    Vec3 axis = -Vec3::UnitZ(); // unit, pointing into the material
    double diameter = 5;
    double depth = 1e9;         // blind hole (to the shoulder, if it has a point); larger than the material: through
    double counterbore_diameter = 0, counterbore_depth = 0;   // 0: none
    double countersink_diameter = 0, countersink_angle_deg = 90;  // 0: none; the included angle
    double point_angle_deg = 0;  // a drill point's included angle at a blind hole's bottom (118 for a drill); 0: flat
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

// A 60 x 40 x 12 mm plate with hole forms, all scanned to the bottom:
//   x = -18: 6.6 mm through hole, counterbored 11 mm x 6.5 mm deep (an M6 socket head screw);
//   x = 0:   4.5 mm through hole, countersunk 9 mm at 90 degrees;
//   x = 18:  5 mm blind hole 8 mm deep (to the shoulder) with a 118 degree drill point.
[[nodiscard]] PartSpec hole_forms_spec();

// A 50 x 30 x 10 mm block whose top is a 4 mm high dome (a freeform face), scanned all round. Its volume is
// 15000 + 4 x 25 x 15 x 16 / 9 = 17666.67 mm^3.
[[nodiscard]] PartSpec domed_block_spec();

}  // namespace einstar::fit
