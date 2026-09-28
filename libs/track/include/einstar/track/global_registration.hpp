#pragma once

// Pose-independent registration of a frame against the model: FPFH descriptors,
// feature matching, RANSAC with edge-length pre-checks, rigid fit. Used to relocalise
// after tracking loss when the scanner comes back from anywhere.

#include <cstdint>
#include <optional>
#include <vector>

#include "einstar/core/se3.hpp"

namespace einstar::track {

struct OrientedCloud {
    std::vector<Vec3f> points;
    std::vector<Vec3f> normals;
};

using Fpfh = std::array<float, 33>;

[[nodiscard]] OrientedCloud voxel_downsample(const OrientedCloud& in, float voxel_mm);
[[nodiscard]] std::vector<Fpfh> compute_fpfh(const OrientedCloud& cloud, float radius_mm);

// Precomputed model side (rebuild when the model has grown noticeably).
class FeatureModel {
public:
    FeatureModel();
    FeatureModel(OrientedCloud cloud, float feature_radius_mm);
    ~FeatureModel();
    FeatureModel(FeatureModel&&) noexcept;
    FeatureModel& operator=(FeatureModel&&) noexcept;

    [[nodiscard]] const OrientedCloud& cloud() const { return cloud_; }
    [[nodiscard]] const std::vector<Fpfh>& features() const { return features_; }
    [[nodiscard]] bool empty() const { return cloud_.points.empty(); }

    // k nearest model features to `f` (indices).
    void nearest_features(const Fpfh& f, int k, std::vector<std::uint32_t>& out) const;
    // Nearest model point within `radius` (for inlier counting).
    [[nodiscard]] std::optional<std::uint32_t> nearest_point(const Vec3f& p, float radius_mm) const;

private:
    struct Index;
    OrientedCloud cloud_;
    std::vector<Fpfh> features_;
    std::unique_ptr<Index> index_;
};

struct GlobalRegistrationParams {
    float voxel_mm = 2.0f;
    float feature_radius_mm = 10.0f;
    int ransac_iterations = 20000;
    float inlier_distance_mm = 2.0f;
    float edge_ratio = 0.9f;
    int min_inliers = 60;
    // Symmetric parts: a very different pose fitting nearly as well makes the match ambiguous.
    int hypotheses = 4;                   // distinct RANSAC hypotheses scored against the whole model
    double distinct_mm = 15.0;            // hypotheses farther apart than this (or...)
    double distinct_deg = 8.0;            // ... rotated more than this are different poses
    double ambiguity_ratio = 0.85;        // second-best fit / best fit above this = ambiguous
};

struct GlobalRegistrationCandidate {
    SE3 T_model_frame = SE3::Identity();  // frame coords -> model (world) coords
    int inliers = 0;
    double fitness = 0;                   // inliers / frame samples
};

struct GlobalRegistrationResult {
    SE3 T_model_frame = SE3::Identity();  // best candidate
    int inliers = 0;
    double fitness = 0;
    bool ambiguous = false;
    // Every distinct candidate that fits nearly as well as the best (best first).
    std::vector<GlobalRegistrationCandidate> candidates;
};

[[nodiscard]] std::optional<GlobalRegistrationResult> register_global(const OrientedCloud& frame_cloud,
                                                                      const FeatureModel& model,
                                                                      const GlobalRegistrationParams& params,
                                                                      std::uint32_t seed = 1);

}  // namespace einstar::track
