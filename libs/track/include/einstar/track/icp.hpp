#pragma once

// Projective point-to-plane ICP against a raycast model view, with robust (Huber) weights,
// a weak prior towards the predicted pose (keeps flat / slippery surfaces from sliding),
// and degeneracy analysis from the Gauss-Newton Hessian.

#include <functional>
#include <vector>

#include "einstar/track/frame.hpp"
#include "einstar/track/tsdf.hpp"

namespace einstar::track {

// A marker observed in the frame (camera coordinates) associated with a map marker (world).
struct IcpMarker {
    Vec3 p_camera;
    Vec3 q_world;
};

struct IcpParams {
    int levels = 3;                       // source subsampling 4, 2, 1
    std::array<int, 3> iterations{15, 10, 8};
    float max_distance_mm = 3.0f;         // association gate at the finest level (scaled up at coarse levels)
    float max_normal_angle_deg = 35.0f;
    float huber_mm = 0.3f;                // at the finest level; scaled with the level like the gate
    // Normal-space balancing: weight each point by (count of its normal bin)^-alpha so a dominant
    // plane cannot outvote the geometry that actually constrains the motion. 0 disables.
    double normal_balance_alpha = 1.0;
    // Weak prior towards the initial pose, expressed as motion-model uncertainty per frame versus
    // the depth noise. It only matters along directions the geometry cannot constrain.
    double prior_sigma_mm = 5.0;
    double prior_sigma_deg = 2.0;
    double data_sigma_mm = 0.2;
    // Degeneracy-aware update: eigen-directions of the (unit-scaled) Hessian weaker than this
    // fraction of the strongest are not updated from the data; they keep the prediction.
    double degenerate_direction_ratio = 5e-4;  // chosen on mustang replays (full, skip 2, other starts)
    // Optional point-to-point marker terms solved jointly with the surface. Each marker counts like
    // `marker_weight` unit-weight surface correspondences, so markers pin the directions the surface
    // cannot (sliding on symmetric parts) without overriding well-constrained geometry.
    std::vector<IcpMarker> markers;
    double marker_weight = 200.0;
};

struct IcpResult {
    SE3 T_world_camera = SE3::Identity();
    int correspondences = 0;
    int candidates = 0;                   // valid source points considered at the finest level
    double rms_mm = 0;                    // robust RMS of point-to-plane residuals (inliers)
    double inlier_ratio = 0;              // inliers / source points that landed on the model (consistency)
    double coverage = 0;                  // source points that landed on the model / all valid source points
    double min_eigenvalue_ratio = 0;      // smallest / largest Hessian eigenvalue (degeneracy indicator)
    int degenerate_directions = 0;        // directions held by the prediction in the final solve
    Mat6 degenerate_basis = Mat6::Zero(); // first `degenerate_directions` columns, in unit-scaled twist coords about `center`
    Vec3 center = Vec3::Zero();           // linearisation point (world): centroid of the observed surface
    double marker_rms_mm = 0;             // residual of the marker terms at the final pose (0 if none)
    bool converged = false;
};

// Linearisation centre used by the ICP solvers: centroid of (subsampled) frame points at the given pose.
[[nodiscard]] Vec3 icp_center(const DepthFrame& frame, const SE3& T_world_camera);
[[nodiscard]] Vec6 twist_from_center(const Vec6& twist_about_center, const Vec3& c);  // -> twist about the origin
[[nodiscard]] Vec6 twist_to_center(const Vec6& twist_about_origin, const Vec3& c);

// Per-pixel normal-space balancing weights (empty when alpha <= 0).
[[nodiscard]] std::vector<float> normal_balance_weights(const DepthFrame& frame, double alpha);

// Solver signature shared by the CPU implementation and GPU backends.
using IcpFunction = std::function<IcpResult(const DepthFrame&, const RaycastResult&, const SE3&, const SE3&, const IcpParams&)>;

// `model` must be a raycast from the predicted pose (it defines the association camera).
[[nodiscard]] IcpResult icp_point_to_plane(const DepthFrame& frame, const RaycastResult& model, const SE3& T_model_camera,
                                           const SE3& T_init, const IcpParams& params);

}  // namespace einstar::track
