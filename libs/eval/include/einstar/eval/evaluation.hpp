#pragma once

// Evaluation against references (EXStar's poses and meshes): used by the CLI and tuning runs, never
// by the scanning or processing code itself.

#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <ostream>
#include <vector>

#include "einstar/core/se3.hpp"
#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/recon/mesh.hpp"
#include "einstar/session/session.hpp"

namespace einstar::eval {

[[nodiscard]] double percentile(std::vector<double> v, double p);

// Pose files: one line per frame, "<index> r00 r01 r02 t0 r10 ... t2".
void write_pose(std::ostream& out, std::size_t index, const SE3& T);
[[nodiscard]] std::map<std::size_t, SE3> read_poses(const std::filesystem::path& path);

struct Stats {
    double median = 0, p95 = 0;
};

struct PoseComparison {
    std::size_t frames = 0;
    Stats live_mm, live_deg, processed_mm, processed_deg;
    int bad_live_only = 0, bad_processed_only = 0, bad_both = 0;  // > gross_mm from the reference
    // After the best rigid alignment of camera centres (the process step fixes only the gauge).
    Stats aligned_live_mm, aligned_processed_mm;
    std::size_t recovered = 0;  // frames untracked live but posed by processing
    Stats recovered_mm;
    int recovered_bad = 0;
};

// `processed` are the process step's poses by session frame; `reference` is keyed by the frame's
// recorded index.
[[nodiscard]] PoseComparison compare_poses(const session::SessionReader& s, const std::map<std::size_t, SE3>& processed,
                                           const std::map<std::size_t, SE3>& reference, double gross_mm = 3.0);

// Spread of each identified marker's world positions (how consistent a set of poses is).
[[nodiscard]] Stats marker_spread(const session::SessionReader& s, const std::function<std::optional<SE3>(std::size_t)>& pose_of);

struct MeshComparison {
    Stats accuracy_mm;                  // our surface -> reference
    double accuracy_p90_mm = 0;
    double beyond_fraction = 0;         // of our surface farther than the search radius (not in the reference)
    double completeness_05 = 0, completeness_1 = 0;  // of the reference within 0.5 / 1 mm of our surface
};
[[nodiscard]] MeshComparison compare_mesh(const recon::TriangleMesh& mesh, const fixtures::Mesh& reference, float radius_mm = 3.0f);

}  // namespace einstar::eval
