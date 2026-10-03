#pragma once

// GPU-resident TSDF volume (voxel hashing) implementing track::Volume on Metal.

#include <memory>

#include "einstar/core/error.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/track/tsdf.hpp"

namespace einstar::track_metal {

struct MetalTsdfOptions {
    std::uint32_t brick_capacity = 196608;  // 8^3 bricks; 2 KiB each (half sdf + half weight)
    std::uint32_t table_size = 1u << 20;    // hash slots (power of two, >= 2x brick capacity)
};

class MetalTsdfVolume final : public track::Volume {
public:
    static Result<std::unique_ptr<MetalTsdfVolume>> create(std::shared_ptr<gpu::Context> ctx, track::TsdfParams params = {},
                                                           MetalTsdfOptions options = {});
    ~MetalTsdfVolume() override;

    void integrate(const track::DepthFrame& frame, const SE3& T_world_camera, float weight_scale = 1.0f,
                   bool extend_only = false) override;
    [[nodiscard]] track::RaycastResult raycast(const SE3& T_world_camera, const track::Intrinsics& k) const override;
    [[nodiscard]] std::vector<track::SurfacePoint> extract_points(std::uint32_t since_frame = 0, float min_weight = 0.5f,
                                                                  bool canonical = true) const override;
    [[nodiscard]] std::vector<track::BrickCoord> bricks_updated_since(std::uint32_t frame) const override;
    [[nodiscard]] std::vector<track::SurfacePoint> extract_points(const std::vector<track::BrickCoord>& bricks,
                                                                  float min_weight = 0.5f) const override;
    [[nodiscard]] bool fast_full_extraction() const override { return true; }

    void for_each_brick(const BrickVisitor& fn) const override;
    [[nodiscard]] std::size_t brick_count() const override;
    [[nodiscard]] std::uint32_t frame_counter() const override { return frame_; }
    [[nodiscard]] const track::TsdfParams& params() const override { return params_; }
    void clear() override;
    track::ErasedVoxels erase(const LassoSelection& selection) override;
    void restore(const track::ErasedVoxels& erased) override;

    [[nodiscard]] bool pool_exhausted() const;

    // Whole surface as render::PointVertex records in a fresh GPU buffer (drawn without copies).
    struct RenderPoints {
        gpu::Ref<MTL::Buffer> buffer;
        std::size_t count = 0;
    };
    [[nodiscard]] RenderPoints extract_render_points(float min_weight = 0.5f) const;

private:
    struct Impl;
    MetalTsdfVolume(std::unique_ptr<Impl> impl, track::TsdfParams params);
    std::unique_ptr<Impl> impl_;
    track::TsdfParams params_;
    std::uint32_t frame_ = 0;
};

}  // namespace einstar::track_metal
