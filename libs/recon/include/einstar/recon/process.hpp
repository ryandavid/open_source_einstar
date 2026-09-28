#pragma once

// The process step: recorded scan session -> globally consistent poses -> re-fused model -> mesh.
//
// 1. Fragments: consecutive tracked frames are grouped (and split at tracking gaps); each becomes an
//    oriented point cloud in its first frame's coordinates, plus its identified markers.
// 2. Pose graph over every tracked frame: chain edges between consecutive frames (the live relative
//    pose, weighted by live tracking accuracy), registrations between overlapping fragments
//    (consecutive ones and loop closures, as edges between the fragments' middle frames), and markers
//    as landmarks observed by each frame (fixed when a global-marker map was used). Inconsistent
//    loop closures are pruned. Drift is spread smoothly along the chain instead of per fragment.
// 3. Every tracked frame is re-fused with its corrected pose, the surface is meshed (surface nets),
//    small disconnected pieces are removed and the mesh is optionally smoothed.

#include <atomic>
#include <functional>
#include <map>
#include <string>

#include "einstar/core/error.hpp"
#include "einstar/recon/mesh.hpp"
#include "einstar/recon/registration.hpp"
#include "einstar/session/session.hpp"

namespace einstar::recon {

struct ProcessParams {
    // Fragments
    int fragment_frames = 40;
    double fragment_max_gap_s = 0.5;  // a longer gap between tracked frames starts a new fragment
    int cloud_stride_px = 2;
    float cloud_voxel_mm = 1.0f;
    // Pose graph
    bool optimize_poses = true;
    double chain_sigma_deg = 0.15;   // live frame-to-frame accuracy (per frame; generous: drift is systematic)
    double chain_sigma_mm = 0.3;
    double chain_gap_weight = 1e-2;  // chain edges across a tracking gap (relocalisation) are weaker
    // Optional prior of every frame towards its live pose (0 = off). It anchors directions the
    // geometry cannot constrain, but also resists genuine drift correction.
    double prior_sigma_deg = 1.0;
    double prior_sigma_mm = 0.0;
    // Fragments are rebuilt from the optimised poses and the graph re-solved until poses settle.
    int graph_iterations = 2;
    double graph_tolerance_mm = 0.05;
    RegistrationParams registration;
    double loop_min_fitness = 0.3;
    double loop_max_rms_mm = 0.35;
    double loop_min_eigen_ratio = 2e-4;     // weak directions are down-weighted by the information matrix anyway
    double loop_max_correction_mm = 25.0;   // a loop closure far from the live estimate is not trusted
    double loop_max_correction_deg = 10.0;
    double loop_min_overlap = 0.25;         // quick overlap test before registration
    int loop_rounds = 4;                    // search again after each solve (poses improve)
    double prune_chi = 25.0;               // whitened loop-closure error (~0.5 mm with the default information scale)
    // Frame refinement: after the pose graph, every frame is registered against the model built from
    // all fragments (removes drift inside fragments). Skipped for frames whose geometry does not
    // constrain the pose.
    int refine_rounds = 0;  // measured neutral on real scans; costs ~2 ms per frame
    int refine_stride_px = 4;
    double refine_min_fitness = 0.6;
    double refine_max_rms_mm = 0.35;
    double refine_min_eigen_ratio = 4e-3;
    double refine_max_correction_mm = 5.0;
    // Islands (segments with no verified link to the main model): excluded from fusion when this much
    // of them overlaps the main model and less than this fraction of the overlap agrees with it.
    double island_min_overlap = 0.2;
    double island_min_agreement = 0.6;
    bool use_markers = true;
    double marker_sigma_mm = 0.08;
    double marker_consistency_mm = 3.0;     // observations this far from their landmark are dropped
    // Fusion and meshing
    track::TsdfParams tsdf;                 // voxel size etc. of the final model
    bool use_gpu = true;
    float degenerate_weight = 0.5f;
    ExtractParams extract;
    CleanupParams cleanup;
    int smooth_iterations = 0;

    std::function<void(const std::string& stage, double fraction)> progress;
    const std::atomic<bool>* cancel = nullptr;
};

struct ProcessReport {
    int frames_used = 0;
    int fragments = 0;
    int odometry_edges = 0;
    int loop_candidates = 0;
    int loop_edges = 0;          // accepted by registration
    int loop_edges_pruned = 0;   // removed by the pose graph
    int marker_landmarks = 0;
    int marker_observations = 0;
    double max_correction_mm = 0, max_correction_deg = 0;  // largest change of a frame pose
    double median_correction_mm = 0;
    int frames_refined = 0;       // by frame-to-model registration (last round)
    int graph_iterations = 0;
    int islands = 0;              // segments with no verified link to the main model
    int islands_excluded = 0;     // ... that contradicted it and were left out of the fusion
    int frames_excluded = 0;
    std::size_t vertices = 0, triangles = 0;
    CleanupReport cleanup;
    std::map<std::string, double> stage_ms;
};

struct ProcessResult {
    TriangleMesh mesh;
    std::map<std::size_t, SE3> frame_poses;  // session frame index -> optimised T_world_camera
    ProcessReport report;
};

[[nodiscard]] Result<ProcessResult> process_session(const session::SessionReader& session, const ProcessParams& params = {});

}  // namespace einstar::recon
