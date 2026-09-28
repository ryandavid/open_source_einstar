#pragma once

// Sparse TSDF volume on a hash of 8^3 voxel bricks (CPU reference; the Metal port mirrors it).

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "einstar/track/frame.hpp"

namespace einstar::track {

inline constexpr int kBrickSize = 8;
inline constexpr int kBrickVoxels = kBrickSize * kBrickSize * kBrickSize;

struct BrickCoord {
    int x = 0, y = 0, z = 0;
    friend bool operator==(const BrickCoord&, const BrickCoord&) = default;
};

struct BrickCoordHash {
    std::size_t operator()(const BrickCoord& c) const noexcept {
        return static_cast<std::size_t>(c.x) * 73856093u ^ static_cast<std::size_t>(c.y) * 19349669u ^
               static_cast<std::size_t>(c.z) * 83492791u;
    }
};

struct Brick {
    std::array<float, kBrickVoxels> sdf;     // normalised to [-1, 1] (units of truncation)
    std::array<float, kBrickVoxels> weight;
    std::uint32_t last_update = 0;           // frame counter
    Brick() {
        sdf.fill(1.0f);
        weight.fill(0.0f);
    }
};

struct TsdfParams {
    float voxel_mm = 0.5f;
    float truncation_mm = 2.5f;
    float max_weight = 64.0f;
    float min_depth_mm = 150.0f;
    float max_depth_mm = 700.0f;
};

struct RaycastResult {
    Intrinsics intrinsics;
    Image<Vec3f> points;   // world frame, z-component irrelevant for validity; see `valid`
    Image<Vec3f> normals;  // world frame
    Image<std::uint8_t> valid;
};

struct SurfacePoint {
    Vec3f position;
    Vec3f normal;
    float weight;
};

class TsdfVolume {
public:
    explicit TsdfVolume(TsdfParams params = {});

    // Fuses a frame observed from camera pose T_world_camera.
    // `extend_only`: only voxels never observed before are written (poses of weakly-constrained frames
    // may extend the model but must not reshape it).
    void integrate(const DepthFrame& frame, const SE3& T_world_camera, float weight_scale = 1.0f, bool extend_only = false);

    // Renders the model from a camera (vertex + normal maps in world coordinates).
    [[nodiscard]] RaycastResult raycast(const SE3& T_world_camera, const Intrinsics& k) const;

    // Zero-crossing surface points of bricks touched since `since_frame` (for incremental display).
    [[nodiscard]] std::vector<SurfacePoint> extract_points(std::uint32_t since_frame = 0, float min_weight = 0.5f) const;
    [[nodiscard]] std::vector<BrickCoord> bricks_updated_since(std::uint32_t frame) const;
    [[nodiscard]] std::vector<SurfacePoint> extract_points(const std::vector<BrickCoord>& bricks, float min_weight = 0.5f) const;

    [[nodiscard]] std::size_t brick_count() const;
    [[nodiscard]] std::uint32_t frame_counter() const { return frame_counter_; }
    [[nodiscard]] const TsdfParams& params() const { return params_; }
    void clear();

    // Trilinear SDF sample in mm (nullopt if unobserved).
    [[nodiscard]] std::optional<float> sample_sdf(const Vec3f& world) const;

private:
    [[nodiscard]] BrickCoord brick_of(const Vec3f& world) const;
    [[nodiscard]] const Brick* find(const BrickCoord& c) const;
    [[nodiscard]] bool voxel(const Vec3f& world, float& sdf, float& weight) const;  // nearest voxel
    [[nodiscard]] bool trilinear(const Vec3f& world, float& sdf) const;

    TsdfParams params_;
    float brick_mm_;
    mutable std::shared_mutex mutex_;  // integrate (writer) vs raycast/extract (readers)
    std::unordered_map<BrickCoord, std::unique_ptr<Brick>, BrickCoordHash> bricks_;
    std::uint32_t frame_counter_ = 0;
};

}  // namespace einstar::track
