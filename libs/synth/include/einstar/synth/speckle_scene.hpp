#pragma once

// Synthetic IR speckle renderer: ray-casts analytic scenes lit by a dot-pattern projector
// and produces camera images plus ground-truth depth. Used to test the depth pipeline
// (and to feed the device emulator) without hardware.

#include <cstdint>
#include <optional>
#include <random>
#include <variant>
#include <vector>

#include "einstar/core/camera.hpp"
#include "einstar/core/image.hpp"

namespace einstar::synth {

struct Plane {
    Vec3 point;
    Vec3 normal;
};
struct Sphere {
    Vec3 center;
    double radius = 0;
};
struct Box {  // oriented box: T_world_box maps box-local coords (centred) to world
    SE3 T_world_box = SE3::Identity();
    Vec3 half_extent{10, 10, 10};
};
using Primitive = std::variant<Plane, Sphere, Box>;

struct Hit {
    double t = 0;
    Vec3 point;
    Vec3 normal;
};

// Retro-reflective marker sticker lying on a surface: a bright disc inside a dark ring. Lit by the
// scanner's strobe, it appears saturated in both IR cameras and hides the speckle underneath.
struct Marker {
    Vec3 center;
    Vec3 normal;
    double diameter = 6.0;        // bright disc, mm
    double ring_diameter = 10.0;  // dark surround, mm
};

struct Scene {
    std::vector<Primitive> primitives;  // world coordinates, mm
    std::vector<Marker> markers;

    [[nodiscard]] std::optional<Hit> intersect(const Vec3& origin, const Vec3& dir) const;
};

// Random dot pattern emitted by the projector, defined on its image plane.
struct DotPattern {
    int width = 1280;
    int height = 800;
    ImageF32 intensity;  // 0..1

    static DotPattern random(int width, int height, int num_dots, double sigma_px, std::uint32_t seed);
    [[nodiscard]] float sample(double u, double v) const;  // bilinear, 0 outside
};

struct Projector {
    CameraModel model;                  // pinhole (distortion ignored)
    SE3 T_world_projector = SE3::Identity();
    DotPattern pattern;
    double power = 1.0;
};

struct RenderParams {
    double ambient = 0.04;
    double albedo = 0.9;
    double noise_sigma = 2.0;  // grey levels
    double marker_brightness = 1.3;  // retro-reflective return (saturates)
    double marker_ring = 0.02;
    int supersample = 2;       // per axis
    std::uint32_t seed = 1;
};

struct RenderedView {
    ImageU8 image;
    ImageF32 depth;  // camera-frame z in mm, 0 where no surface
};

// Renders a view through the camera model, including lens distortion (pixel centres at integer coords).
[[nodiscard]] RenderedView render_view(const Scene& scene, const Projector& projector, const CameraModel& camera,
                                       const SE3& T_world_camera, const RenderParams& params);

}  // namespace einstar::synth
