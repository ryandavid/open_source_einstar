#pragma once

// Point-to-plane ICP on Metal with the same semantics as track::icp_point_to_plane. Every
// iteration of every level is encoded into a single command buffer (no per-iteration CPU sync).

#include <memory>

#include "einstar/core/error.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/track/icp.hpp"

namespace einstar::track_metal {

class MetalIcp {
public:
    static Result<std::unique_ptr<MetalIcp>> create(std::shared_ptr<gpu::Context> ctx);
    ~MetalIcp();

    [[nodiscard]] track::IcpResult solve(const track::DepthFrame& frame, const track::RaycastResult& model,
                                         const SE3& T_model_camera, const SE3& T_init, const track::IcpParams& params);

    // Adapter usable as track::IcpFunction (the object must outlive the function).
    [[nodiscard]] track::IcpFunction as_function();

    [[nodiscard]] double last_gpu_ms() const { return last_gpu_ms_; }
    [[nodiscard]] std::uint32_t last_iterations() const { return last_iterations_; }  // solves that ran

private:
    struct Impl;
    explicit MetalIcp(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
    double last_gpu_ms_ = 0;
    std::uint32_t last_iterations_ = 0;
};

}  // namespace einstar::track_metal
