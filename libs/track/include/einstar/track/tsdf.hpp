#pragma once

// Sparse TSDF volume on a hash of 8^3 voxel bricks (CPU reference; the Metal port mirrors it).

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <span>
#include <memory>
#include <optional>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "einstar/core/lasso.hpp"
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
    std::array<std::uint8_t, kBrickVoxels> observations{};  // frames that saw a surface here (TsdfParams::count_observations)
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
    // Count, per voxel, the frames that observed a surface within the truncation band (saturating at
    // 255). Offline processing uses it to keep only surface seen by several frames: a flake from one
    // or two frames' noise can carry a high weight, but never many observations.
    bool count_observations = false;
};

struct RaycastResult {
    Intrinsics intrinsics;
    mutable Image<Vec3f> points;   // world frame, z-component irrelevant for validity; see `valid`
    mutable Image<Vec3f> normals;  // world frame
    mutable Image<std::uint8_t> valid;
    std::shared_ptr<const DeviceRaycastData> device;  // GPU volumes: CPU images filled on demand

    void ensure_cpu() const;
};

struct SurfacePoint {
    Vec3f position;
    Vec3f normal;
    float weight;
};

// Orders points by position so results that depend on order (voxel averaging, RANSAC sampling) do not
// depend on how they were produced (GPU extraction appends in a varying order).
void sort_canonical(std::vector<SurfacePoint>& points);

// Voxels an erase cleared, as they were (to undo it): global voxel coordinates (brick * 8 + local).
struct ErasedVoxels {
    std::vector<Eigen::Vector3i> voxel;
    std::vector<float> sdf, weight;  // sdf normalised as in Brick
    [[nodiscard]] std::size_t size() const { return voxel.size(); }
};

// Fused surface model. Implemented on the CPU (TsdfVolume, the reference) and in Metal.
class Volume {
public:
    virtual ~Volume() = default;

    // Fuses a frame observed from camera pose T_world_camera.
    // `extend_only`: only voxels never observed before are written (poses of weakly-constrained frames
    // may extend the model but must not reshape it).
    virtual void integrate(const DepthFrame& frame, const SE3& T_world_camera, float weight_scale = 1.0f,
                           bool extend_only = false) = 0;

    // Renders the model from a camera (vertex + normal maps in world coordinates).
    [[nodiscard]] virtual RaycastResult raycast(const SE3& T_world_camera, const Intrinsics& k) const = 0;

    // Zero-crossing surface points of bricks touched since `since_frame` (0 = everything), in a
    // canonical order unless `canonical` is false (then sort_canonical() them before any use that
    // depends on order, e.g. off the calling thread).
    [[nodiscard]] virtual std::vector<SurfacePoint> extract_points(std::uint32_t since_frame = 0, float min_weight = 0.5f,
                                                                   bool canonical = true) const = 0;
    [[nodiscard]] virtual std::vector<BrickCoord> bricks_updated_since(std::uint32_t frame) const = 0;
    [[nodiscard]] virtual std::vector<SurfacePoint> extract_points(const std::vector<BrickCoord>& bricks, float min_weight = 0.5f) const = 0;
    // True when extracting the whole surface is cheap enough to do for every display refresh.
    [[nodiscard]] virtual bool fast_full_extraction() const { return false; }

    // Visits every allocated brick (offline meshing). `sdf` is normalised to [-1, 1] of the
    // truncation distance, voxel (x, y, z) at index (z * 8 + y) * 8 + x, centre at
    // ((coord * 8 + xyz) + 0.5) * voxel_mm.
    // `observations` is empty unless TsdfParams::count_observations.
    using BrickVisitor = std::function<void(const BrickCoord&, std::span<const float> sdf, std::span<const float> weight,
                                            std::span<const std::uint8_t> observations)>;
    virtual void for_each_brick(const BrickVisitor& fn) const = 0;

    [[nodiscard]] virtual std::size_t brick_count() const = 0;
    [[nodiscard]] virtual std::uint32_t frame_counter() const = 0;
    [[nodiscard]] virtual const TsdfParams& params() const = 0;
    virtual void clear() = 0;

    // Clears every observed voxel whose centre `selection` contains -- the surface and its truncation
    // band, so neither extraction nor raycasting finds it -- and returns what they held. Bricks stay
    // allocated and later frames fuse into them as usual.
    virtual ErasedVoxels erase(const LassoSelection& selection) = 0;
    // Puts erased voxels back (undo; nothing may have been fused since).
    virtual void restore(const ErasedVoxels& erased) = 0;
};

class TsdfVolume final : public Volume {
public:
    explicit TsdfVolume(TsdfParams params = {});

    void integrate(const DepthFrame& frame, const SE3& T_world_camera, float weight_scale = 1.0f, bool extend_only = false) override;
    [[nodiscard]] RaycastResult raycast(const SE3& T_world_camera, const Intrinsics& k) const override;
    [[nodiscard]] std::vector<SurfacePoint> extract_points(std::uint32_t since_frame = 0, float min_weight = 0.5f,
                                                           bool canonical = true) const override;
    [[nodiscard]] std::vector<BrickCoord> bricks_updated_since(std::uint32_t frame) const override;
    [[nodiscard]] std::vector<SurfacePoint> extract_points(const std::vector<BrickCoord>& bricks, float min_weight = 0.5f) const override;

    void for_each_brick(const BrickVisitor& fn) const override;
    [[nodiscard]] std::size_t brick_count() const override;
    [[nodiscard]] std::uint32_t frame_counter() const override { return frame_counter_; }
    [[nodiscard]] const TsdfParams& params() const override { return params_; }
    void clear() override;
    ErasedVoxels erase(const LassoSelection& selection) override;
    void restore(const ErasedVoxels& erased) override;

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
