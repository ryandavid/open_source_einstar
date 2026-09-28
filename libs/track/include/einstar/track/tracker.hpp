#pragma once

// Frame-to-model tracker: motion prediction, robust ICP against the fused TSDF, explicit
// acceptance tests, lost/relocalisation state. Every frame gets a verdict; nothing is dropped
// silently (rejected frames are reported with a reason).

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "einstar/markers/marker_map.hpp"
#include "einstar/track/global_registration.hpp"
#include "einstar/track/icp.hpp"
#include "einstar/track/tsdf.hpp"

namespace einstar::track {

enum class TrackState { initializing, tracking, lost, confirming };

// geometry: surface only. hybrid: surface + markers jointly. markers: markers carry the pose (surface
// refines it where it agrees). EXStar's "Feature", "Hybrid" and "Markers" alignment.
enum class AlignMode { geometry, hybrid, markers };

struct TrackerParams {
    AlignMode mode = AlignMode::hybrid;  // hybrid falls back to geometry when a frame has no markers
    markers::MapParams marker_map;
    double marker_weight = 200.0;         // see IcpParams::marker_weight
    double max_marker_rms_mm = 0.5;       // markers must agree with the final pose this well
    int min_marker_inliers = 3;
    TsdfParams tsdf;
    IcpParams icp;
    double model_scale = 0.5;           // raycast resolution relative to the depth frame
    double min_inlier_ratio = 0.7;      // of source points that land on the model (good frames: 0.8-0.9)
    double min_coverage = 0.15;         // of the frame that must overlap the model
    int min_correspondences = 800;
    double max_rms_mm = 0.45;
    double max_speed_mm_s = 800.0;      // generous hand-motion limits, scaled by elapsed time
    double max_rot_speed_deg_s = 240.0;
    // Below this the pose is only weakly determined by geometry (e.g. surfaces of revolution): the
    // frame is tracked but only extends the model into unobserved space.
    double degenerate_eigen_ratio = 5e-3;  // (centred, unit-scaled Hessian; ~7% of frames on real scans)
    float degenerate_weight = 0.5f;
    int relocalize_attempts_per_frame = 2;
    // A relocalisation is only trusted after this many consecutive frames track consistently from
    // it, under stricter thresholds; nothing is fused into the model until then.
    int confirm_frames = 3;
    double reloc_min_inlier_ratio = 0.75;
    double reloc_min_coverage = 0.35;
    double reloc_min_eigen_ratio = 5e-3;   // relocalising onto ambiguous (sliding) geometry is refused
    // Global (pose-independent) relocalisation while lost.
    bool global_relocalization = true;
    bool fuse_surface = true;              // false: track only (e.g. global-marker capture)
    int global_reloc_every = 3;            // attempt on every Nth lost frame (it costs ~50-150 ms)
    GlobalRegistrationParams global;
    double feature_model_rebuild_growth = 0.15;  // rebuild descriptors when the model grew by 15%
};

struct TrackResult {
    TrackState state = TrackState::initializing;
    SE3 T_world_camera = SE3::Identity();
    IcpResult icp;
    bool accepted = false;
    bool integrated = false;
    bool degenerate = false;
    bool relocalized = false;
    int markers_seen = 0;       // stereo markers in the frame
    int markers_matched = 0;    // associated with the map at the final pose
    bool marker_pose = false;   // the pose was seeded/verified by markers
    std::vector<std::pair<int, int>> marker_ids;  // (frame marker index, map id) at the final pose
    std::string reason;  // why a frame was rejected
    double ms = 0;
};

class Tracker {
public:
    // Uses the CPU TSDF unless a volume (e.g. the Metal one) is supplied.
    explicit Tracker(TrackerParams params = {}, std::unique_ptr<Volume> volume = nullptr);

    TrackResult process(const DepthFrame& frame);
    // Replace the ICP solver (e.g. with the Metal implementation). Must have the same semantics.
    void set_icp_solver(IcpFunction f) { icp_ = std::move(f); }

    // Seed the pose of the first frame (e.g. from markers); otherwise identity.
    void set_initial_pose(const SE3& T) { initial_pose_ = T; }

    [[nodiscard]] Volume& volume() { return *volume_; }
    [[nodiscard]] const Volume& volume() const { return *volume_; }
    [[nodiscard]] TrackState state() const { return state_; }
    [[nodiscard]] const markers::MarkerMap& marker_map() const { return map_; }
    // Replace the marker map (e.g. an optimised global-marker map, markers flagged fixed).
    void set_marker_map(std::vector<markers::MapMarker> m) { map_.set_markers(std::move(m)); }
    [[nodiscard]] const SE3& last_good_pose() const { return last_pose_; }
    void reset();

private:
    [[nodiscard]] SE3 predict(double t) const;
    [[nodiscard]] std::optional<SE3> global_candidate(const DepthFrame& frame);
    [[nodiscard]] std::optional<std::string> check(const IcpResult& r, const SE3& from, double dt, bool strict) const;

    TrackerParams params_;
    std::unique_ptr<Volume> volume_;
    markers::MarkerMap map_;
    IcpFunction icp_ = icp_point_to_plane;
    TrackState state_ = TrackState::initializing;
    std::optional<SE3> initial_pose_;
    SE3 last_pose_ = SE3::Identity();
    SE3 prev_pose_ = SE3::Identity();
    double last_time_ = 0, prev_time_ = 0;
    bool have_velocity_ = false;
    int degenerate_dirs_ = 0;              // unobservable directions at the last frame
    Mat6 degenerate_basis_ = Mat6::Zero(); // (unit-scaled twist coordinates about degenerate_center_)
    Vec3 degenerate_center_ = Vec3::Zero();
    int confirm_count_ = 0;
    int lost_frames_ = 0;
    FeatureModel feature_model_;
    std::size_t feature_model_bricks_ = 0;
    std::uint32_t reloc_seed_ = 1;
    SE3 lost_pose_ = SE3::Identity();  // last trusted pose before the loss
};

}  // namespace einstar::track
