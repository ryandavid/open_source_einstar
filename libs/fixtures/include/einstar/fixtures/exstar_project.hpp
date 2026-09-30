#pragma once

// Read-only access to EXStar project data (docs/exstar-project-format.md), used as real-sensor
// test fixtures for tracking/fusion development. Never writes to the project.

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "einstar/core/error.hpp"
#include "einstar/core/image.hpp"
#include "einstar/core/se3.hpp"

namespace einstar::fixtures {

struct MarkerObservation {
    Vec3 position;  // camera frame for per-frame markers, world frame for the global map
    Vec3 normal;
    double diameter = 0;
    double observations = 0;
    int id = -1;
};

struct DepthIntrinsics {
    double fx = 0, fy = 0, cx = 0, cy = 0;
};

struct Frame {
    int id = 0;                      // 1-based
    SE3 T_world_camera = SE3::Identity();  // EXStar's (optimised) pose: camera -> world
    DepthIntrinsics intrinsics;      // for the 640x512 grid
    ImageF32 depth;                  // Z in mm, 0 = none; pixels EXStar flagged as deleted are zeroed
    std::vector<MarkerObservation> markers;
};

class ExstarProject {
public:
    // `path` is a *.ir_E10_prj file or its base path (without extension).
    static Result<std::unique_ptr<ExstarProject>> open(const std::filesystem::path& path);
    ~ExstarProject();

    [[nodiscard]] std::size_t frame_count() const { return blocks_.size(); }
    [[nodiscard]] const std::vector<MarkerObservation>& global_markers() const { return global_markers_; }
    [[nodiscard]] const DepthIntrinsics& fallback_intrinsics() const { return fallback_; }
    [[nodiscard]] double baseline_mm() const { return baseline_; }

    Result<Frame> read_frame(std::size_t index, bool apply_deletion_flags = true) const;
    // Byte range of the frame's block in the .data_base file (offset, size).
    [[nodiscard]] std::pair<std::uint64_t, std::uint64_t> frame_block(std::size_t index) const {
        return {blocks_[index].offset, blocks_[index].size};
    }

private:
    struct BlockRef {
        std::uint64_t offset = 0;
        std::uint64_t size = 0;
    };
    ExstarProject() = default;

    int fd_ = -1;
    std::vector<BlockRef> blocks_;
    std::vector<MarkerObservation> global_markers_;
    DepthIntrinsics fallback_;
    double baseline_ = 0;
};

// Loads a binary or ASCII STL as a triangle soup (used for fixture validation).
struct Mesh {
    std::vector<Vec3f> vertices;          // 3 per triangle
};
[[nodiscard]] Result<Mesh> load_stl(const std::filesystem::path& path);

// Point-to-mesh distance with a uniform grid over triangles (evaluation only).
class MeshDistance {
public:
    explicit MeshDistance(const Mesh& mesh, float cell_mm = 2.0f);
    // Unsigned distance to the closest triangle, or nullopt if none within `max_mm`.
    [[nodiscard]] std::optional<float> distance(const Vec3f& p, float max_mm = 5.0f) const;

private:
    [[nodiscard]] std::int64_t key(int x, int y, int z) const;
    const Mesh& mesh_;
    float cell_;
    std::unordered_map<std::int64_t, std::vector<std::uint32_t>> grid_;
};

// Unprojects a fixture depth map into camera-frame points (every `step` pixels).
[[nodiscard]] std::vector<Vec3f> unproject(const Frame& frame, int step = 1);

}  // namespace einstar::fixtures
