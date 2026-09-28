#pragma once

// Oriented point clouds and pairwise point-to-plane registration (fragment odometry and loop
// closures for the pose graph).

#include <memory>
#include <vector>

#include "einstar/core/se3.hpp"

namespace einstar::recon {

struct Cloud {
    std::vector<Vec3f> points;
    std::vector<Vec3f> normals;  // unit, same size as points

    [[nodiscard]] std::size_t size() const { return points.size(); }
    [[nodiscard]] bool empty() const { return points.empty(); }
};

// Averages points and normals per voxel.
[[nodiscard]] Cloud voxel_downsample(const Cloud& in, float voxel_mm);
[[nodiscard]] Cloud transformed(const Cloud& in, const SE3& T);

// k-d tree over a cloud's points (the cloud must outlive it).
class CloudIndex {
public:
    explicit CloudIndex(const Cloud& cloud);
    ~CloudIndex();
    CloudIndex(CloudIndex&&) noexcept;
    CloudIndex& operator=(CloudIndex&&) noexcept;

    [[nodiscard]] const Cloud& cloud() const { return *cloud_; }
    // Nearest point index within max_mm, or -1.
    [[nodiscard]] int nearest(const Vec3f& p, float max_mm) const;

private:
    struct Impl;
    const Cloud* cloud_;
    std::unique_ptr<Impl> impl_;
};

struct RegistrationParams {
    int iterations = 30;
    float start_distance_mm = 3.0f;  // correspondence gate, shrinking linearly to...
    float final_distance_mm = 1.0f;
    float huber_mm = 0.3f;
    float min_normal_dot = 0.7f;
    float inlier_mm = 0.8f;          // for fitness / rms
    std::size_t max_source_points = 5000;
    // Information matrix scale: noise per correspondence, and a cap on how many correspondences
    // count as independent (neighbouring points share calibration / fusion errors).
    double sigma_mm = 0.15;
    double max_independent = 100;
    float conflict_radius_mm = 4.0f;
    float conflict_distance_mm = 1.5f;
    float conflict_normal_dot = 0.5f;
};

struct RegistrationResult {
    SE3 T_target_source = SE3::Identity();
    double fitness = 0;    // fraction of source points with an inlier correspondence
    double rms_mm = 0;     // of inlier point-to-plane distances
    int correspondences = 0;
    double min_eigen_ratio = 0;  // conditioning of the (centred, unit-scaled) Hessian
    // Of the source points with target geometry nearby (within conflict_radius_mm), the fraction that
    // contradicts it (off the surface or facing another way). A dominant plane can make a wrong
    // alignment fit well; the rest of the geometry then conflicts.
    double conflict = 0;
    // Information of the right perturbation [rotation; translation] of T_target_source (see
    // optim::PoseEdge).
    Mat6 information = Mat6::Zero();
    bool converged = false;
};

[[nodiscard]] RegistrationResult register_point_to_plane(const Cloud& source, const CloudIndex& target, const SE3& initial,
                                                         const RegistrationParams& params = {});

}  // namespace einstar::recon
