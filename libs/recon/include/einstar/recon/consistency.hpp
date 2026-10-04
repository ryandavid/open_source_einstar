#pragma once

// Model-consistency rejection of depth pixels (the process step's second fusion pass).
//
// Stereo depth has a few per cent of pixels far off the surface, almost all along depth borders
// (the foreground's disparity smeared over the background, pixels straddling a step, background
// disparity bleeding onto a foreground edge). Fused, they bias the surface inside the truncation
// band and leave flakes. A first fusion of every frame gives a consensus model; each frame is then
// compared with it from its own camera and the pixels the model contradicts are dropped before the
// final fusion:
// - the model surface the pixel's ray meets faces the camera and the pixel lies in front of it: the
//   pixel floats in space the views that built that surface looked through;
// - ... and the pixel lies behind it: the surface should have hidden it.
// A pixel is kept when it lies near any model surface facing its camera (a silhouette the model
// places slightly differently, a surface behind a flake of the model), when its ray meets no model
// surface (no evidence) and when the surface it meets faces away from the camera (the camera looks
// at the back of a part whose other side the model lacks: the far face of a thin part).

#include <cstddef>
#include <vector>

#include "einstar/core/se3.hpp"
#include "einstar/recon/mesh.hpp"
#include "einstar/recon/registration.hpp"
#include "einstar/track/frame.hpp"

namespace einstar::recon {

struct ConsistencyParams {
    // A pixel agrees with a model surface within this distance of it (along the surface normal): a
    // constant part for pose and calibration residuals, plus stereo depth noise, which grows with the
    // square of the depth (noise_tolerance_mm at reference_depth_mm): 1 mm at 400 mm, 0.5 mm at
    // 250 mm. (Measured on the car display: 0.6 mm at 400 mm cleaned the dark back plate a little more
    // but thinned the glass; 1.5 mm kept more of the tails.)
    float tolerance_mm = 0.3f;
    float noise_tolerance_mm = 0.7f;
    float reference_depth_mm = 400.0f;
    // Along the ray a pixel may be at most this many tolerances from the surface it meets (a point-to-
    // plane distance alone accepts anything along a surface seen edge-on).
    float max_ray_tolerances = 4.0f;
    // A pixel that disagrees with the surface its ray meets is kept if a model surface facing its
    // camera lies within the tolerance (along its normal) and this radius.
    float support_radius_mm = 1.5f;
    // Pixels whose normal differs from that of the model surface they agree with by more than this
    // are dropped too: flying pixels at steps lie on the surface but face sideways (0 = off; the
    // depth normals are noisy, 45 degrees also cost sparse surface on a glossy part).
    double max_normal_angle_deg = 60.0;

    [[nodiscard]] float tolerance(float depth_mm) const {
        const float r = depth_mm / reference_depth_mm;
        return tolerance_mm + noise_tolerance_mm * r * r;
    }
};

struct ConsistencyStats {
    std::size_t pixels = 0;    // valid pixels tested
    std::size_t unseen = 0;    // kept: no model surface facing the camera along the ray
    std::size_t in_front = 0;  // dropped: in front of the model surface the ray meets
    std::size_t behind = 0;    // dropped: behind it
    std::size_t normal = 0;    // dropped: normal disagrees with the model's
    [[nodiscard]] std::size_t rejected() const { return in_front + behind + normal; }
    ConsistencyStats& operator+=(const ConsistencyStats& o) {
        pixels += o.pixels, unseen += o.unseen, in_front += o.in_front, behind += o.behind, normal += o.normal;
        return *this;
    }
};

// The model's nearest surface seen from a camera, per pixel (a z-buffer of the mesh).
struct ModelView {
    ImageF32 depth;           // camera z, 0 = no surface
    Image<Vec3f> normal;      // camera frame, unit, pointing out of the surface (into free space)
    [[nodiscard]] bool front_facing(int x, int y, const track::Intrinsics& k) const {
        const Vec3f r(static_cast<float>((x - k.cx) / k.fx), static_cast<float>((y - k.cy) / k.fy), 1.0f);
        return normal(x, y).dot(r) < 0;
    }
};

class ConsistencyModel {
public:
    // `model`: normals into free space, as extract_mesh makes them. It is rendered after an
    // error-bounded simplification (far fewer triangles, same surface); its vertices are the support.
    explicit ConsistencyModel(const TriangleMesh& model);

    [[nodiscard]] ModelView render(const SE3& T_world_camera, const track::Intrinsics& k) const;
    // Drops the pixels of `frame` (camera frame, with normals) that the model contradicts.
    ConsistencyStats filter(track::DepthFrame& frame, const SE3& T_world_camera, const ConsistencyParams& params) const;

private:
    TriangleMesh render_;
    Cloud vertices_;  // the mesh vertices with their normals
    CloudIndex index_;
};

}  // namespace einstar::recon
