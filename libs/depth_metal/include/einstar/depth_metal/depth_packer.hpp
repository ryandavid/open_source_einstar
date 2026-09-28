#pragma once

// Packs a GPU-resident depth frame into the session file's uncompressed layout (u16 depth in
// 1/50 mm, delta-coded along rows, then u8 confidence) on the GPU. The work is only submitted:
// whoever consumes the bytes waits for it (the recording thread), not the scan pipeline.

#include <cstdint>
#include <memory>
#include <span>

#include "einstar/core/error.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/gpu/device_data.hpp"
#include "einstar/gpu/profile.hpp"

namespace einstar::depth_metal {

class DepthPacker {
public:
    static Result<std::unique_ptr<DepthPacker>> create(std::shared_ptr<gpu::Context> ctx);

    struct Packed {
        gpu::Ref<MTL::CommandBuffer> command;
        gpu::Ref<MTL::Buffer> buffer;  // width * height * 3 bytes once `command` completed
        int width = 0, height = 0;
        // The frame's buffers return to the stereo pool when it is released: keep it until packed.
        std::shared_ptr<const gpu::MetalFrameData> frame;
        void wait() const {
            command->waitUntilCompleted();
            gpu::profile::record("recording/pack depth", command.get());
        }
        [[nodiscard]] std::span<const std::uint8_t> bytes() const {
            return {static_cast<const std::uint8_t*>(buffer->contents()), static_cast<std::size_t>(width * height * 3)};
        }
    };
    [[nodiscard]] std::shared_ptr<Packed> pack(std::shared_ptr<const gpu::MetalFrameData> frame) const;

private:
    DepthPacker() = default;
    std::shared_ptr<gpu::Context> ctx_;
    gpu::Ref<MTL::ComputePipelineState> pso_;
};

}  // namespace einstar::depth_metal
