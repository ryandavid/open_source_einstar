#pragma once

// GPU helper: current-frame overlay (world-space PointVertex records) from a GPU-resident frame.

#include <memory>

#include "einstar/core/se3.hpp"
#include "einstar/gpu/device_data.hpp"
#include "einstar/render/types.hpp"

namespace einstar::pipeline {

class GpuOverlay {
public:
    static std::unique_ptr<GpuOverlay> create(std::shared_ptr<gpu::Context> ctx);

    struct Points {
        gpu::Ref<MTL::Buffer> buffer;
        std::size_t count = 0;
    };
    // Every `step`-th pixel, transformed to world by T_world_camera, in one colour.
    [[nodiscard]] Points frame_points(const gpu::MetalFrameData& frame, const SE3& T_world_camera, int step, render::Rgba8 color);

private:
    std::shared_ptr<gpu::Context> ctx_;
    gpu::Ref<MTL::ComputePipelineState> pso_;
    gpu::Ref<MTL::Buffer> counter_;
};

}  // namespace einstar::pipeline
