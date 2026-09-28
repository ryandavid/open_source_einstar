#pragma once

// World marker map, frame-to-map association, rigid pose from correspondences, and
// pose-independent relocalisation from pairwise-distance signatures.

#include <cstdint>
#include <optional>
#include <random>
#include <unordered_map>
#include <vector>

#include "einstar/core/se3.hpp"

namespace einstar::markers {

struct MapMarker {
    int id = -1;
    Vec3 position = Vec3::Zero();  // world, mm
    double diameter = 0;
    int observations = 0;
    bool fixed = false;             // from a global-marker map (not updated while scanning)
};

struct Correspondence {
    int frame_index = -1;  // index into the frame's marker list
    int map_id = -1;
    Vec3 p_camera;         // frame marker (camera frame)
    Vec3 q_world;          // map marker
};

struct PoseEstimate {
    SE3 T_world_camera = SE3::Identity();
    std::vector<Correspondence> inliers;
    double rms_mm = 0;
};

// Least-squares rigid transform (Horn/Kabsch) mapping p_camera -> q_world. Needs >= 3 non-collinear pairs.
[[nodiscard]] std::optional<SE3> fit_rigid(const std::vector<Correspondence>& pairs);

struct MapParams {
    double match_radius_mm = 5.0;    // association with a predicted pose (EXStar: 5 mm for 6 mm markers)
    double merge_radius_mm = 2.0;    // new observations this close to a map marker update it
    double inlier_mm = 0.8;          // post-fit residual for inliers
    double signature_tolerance_mm = 0.35;  // pairwise-distance match tolerance for relocalisation
    double min_pair_mm = 5.0;
    double max_pair_mm = 400.0;
    int min_inliers = 4;
    double min_inlier_fraction = 0.6;  // of the frame's markers, for relocalisation
    int ransac_iterations = 300;
};

class MarkerMap {
public:
    explicit MarkerMap(MapParams params = {}) : params_(params) {}

    [[nodiscard]] const std::vector<MapMarker>& markers() const { return markers_; }
    [[nodiscard]] std::size_t size() const { return markers_.size(); }
    [[nodiscard]] bool empty() const { return markers_.empty(); }
    [[nodiscard]] const MapParams& params() const { return params_; }
    void clear();

    // Replace the map (e.g. with an optimised global-marker map). `fixed` markers are not moved by updates.
    void set_markers(std::vector<MapMarker> markers);

    [[nodiscard]] std::optional<int> nearest(const Vec3& world, double radius_mm) const;

    // Associate frame markers using a pose guess, then refine the pose on the inliers.
    [[nodiscard]] std::optional<PoseEstimate> track(const std::vector<Vec3>& frame_markers, const SE3& T_guess) const;
    // Pose-independent: triangle hypotheses from pairwise-distance signatures, verified on all markers.
    [[nodiscard]] std::optional<PoseEstimate> relocalize(const std::vector<Vec3>& frame_markers, std::uint32_t seed = 1) const;

    // Fold a tracked frame into the map: matched markers get a running mean, unmatched ones are added.
    void update(const std::vector<Vec3>& frame_markers, const std::vector<double>& diameters, const SE3& T_world_camera,
                const std::vector<Correspondence>& matched);

private:
    [[nodiscard]] std::int64_t cell(const Vec3& p) const;
    void index(int slot);
    void rebuild_pairs() const;

    MapParams params_;
    std::vector<MapMarker> markers_;
    std::unordered_map<std::int64_t, std::vector<int>> grid_;  // cell -> marker slots
    int next_id_ = 0;
    // Pairwise distances of map markers, sorted, for relocalisation (rebuilt lazily).
    struct Pair {
        float d;
        int a, b;
    };
    mutable std::vector<Pair> pairs_;
    mutable bool pairs_dirty_ = true;
};

}  // namespace einstar::markers
