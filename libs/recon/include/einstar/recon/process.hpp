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
    // Registrations are rejected when either fragment's points lie in space the other's camera saw
    // as empty (a dominant plane can make a wrong alignment fit well).
    double max_free_space_violation = 0.1;
    float free_space_tolerance_mm = 3.0f;
    int loop_rounds = 4;                    // search again after each solve (poses improve)
    int loop_max_partners = 10;             // nearest overlapping fragments tried per fragment
    double prune_chi = 25.0;               // whitened loop-closure error (~0.5 mm with the default information scale)
    // Lost-frame recovery: frames live tracking rejected are registered against the final model.
    bool recover_lost_frames = true;
    int recover_stride_px = 4;
    float recover_start_distance_mm = 8.0f;  // wide initial gate: lost frames are often fast motion
    double recover_min_fitness = 0.7;
    double recover_max_rms_mm = 0.3;
    double recover_min_eigen_ratio = 2e-3;   // ambiguous (sliding) geometry is not trusted
    double recover_max_correction_mm = 40.0;
    double recover_max_correction_deg = 15.0;
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
    bool simplify = true;          // error-bounded decimation: flat areas lose triangles, detail stays
    SimplifyParams simplify_params;

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
    int frames_recovered = 0;     // untracked live, registered against the final model
    int graph_iterations = 0;
    int islands = 0;              // segments with no verified link to the main model
    int islands_excluded = 0;     // ... that contradicted it and were left out of the fusion
    int frames_excluded = 0;
    std::size_t vertices = 0, triangles = 0;
    CleanupReport cleanup;
    SimplifyReport simplified;
    std::map<std::string, double> stage_ms;
};

struct ProcessResult {
    TriangleMesh mesh;
    std::map<std::size_t, SE3> frame_poses;  // session frame index -> optimised T_world_camera
    ProcessReport report;
};

[[nodiscard]] Result<ProcessResult> process_session(const session::SessionReader& session, const ProcessParams& params = {});

}  // namespace einstar::recon
