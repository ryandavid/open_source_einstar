#pragma once

// The synthetic world shared by the device emulator, tests and debugging tools: an Einstar-like rig,
// a cluttered table-top, the speckle projector and marker stickers.

#include <cstdint>
#include <vector>

#include "einstar/core/camera.hpp"
#include "einstar/synth/speckle_scene.hpp"

namespace einstar::synth {

// Camera pose looking from `eye` at `target` (x right, y down, z forward; `down` is world down).
[[nodiscard]] SE3 look_at(const Vec3& eye, const Vec3& target, const Vec3& down = Vec3::UnitY());

// Rig with the Einstar's geometry (used when no real calibration is available).
[[nodiscard]] RigCalibration synthetic_einstar_rig();

// Table (plane y = 70, "down" is +y) with spheres and boxes arranged so every view constrains all six
// degrees of freedom.
[[nodiscard]] Scene table_scene();
[[nodiscard]] Projector speckle_projector();

// Marker stickers scattered irregularly over the table, at least `min_spacing_mm` apart.
[[nodiscard]] std::vector<Marker> scatter_markers(int count, std::uint32_t seed, double x_min = -260, double x_max = 120,
                                                  double z_min = -120, double z_max = 170, double min_spacing_mm = 24.0);

}  // namespace einstar::synth
