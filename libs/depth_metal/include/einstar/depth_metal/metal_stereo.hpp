#pragma once

// Metal implementation of the stereo pipeline in libs/depth (same algorithm, same parameters).
// All buffers use shared storage, so results are read by the CPU without copies.

#include <memory>
#include <optional>

#include "einstar/calib/rectify.hpp"
#include "einstar/core/error.hpp"
#include "einstar/depth/stereo.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/gpu/device_data.hpp"

namespace einstar::depth_metal {

struct StereoTimings {
    double gpu_ms = 0;      // GPU execution time of the command buffer
    double speckle_ms = 0;  // CPU speckle filter
    double total_ms = 0;
};

class MetalStereo {
public:
    // `width`/`height`: resolution the disparity is computed at (the finest level).
    static Result<std::unique_ptr<MetalStereo>> create(std::shared_ptr<gpu::Context> ctx, const depth::StereoParams& params,
                                                        int width, int height);
    ~MetalStereo();

    // Optional fused rectification: raw full-res images + full-res remap tables -> half-res rectified input.
    Result<void> set_rectification(const calib::RemapTable& left, const calib::RemapTable& right, int raw_width, int raw_height);

    // Stereo on already rectified images at the configured resolution.
    Result<depth::StereoResult> compute(ImageView<const std::uint8_t> left, ImageView<const std::uint8_t> right);
    // Rectify (to half of the remap resolution) and run stereo; optionally returns the rectified pair.
    Result<depth::StereoResult> compute_raw(ImageView<const std::uint8_t> raw_left, ImageView<const std::uint8_t> raw_right,
                                            ImageU8* rect_left = nullptr, ImageU8* rect_right = nullptr);

    // Geometry for converting disparity to points (disparity resolution; same as depth::RectifiedGeometry).
    void set_point_params(const depth::RectifiedGeometry& geometry, float min_depth_mm, float max_depth_mm,
                          float max_depth_jump_mm = 4.0f);
    // Full GPU path: raw pair -> rectified -> stereo -> speckle filter -> points/normals/weights,
    // all resident on the GPU. Optionally copies out the rectified pair (previews).
    Result<std::shared_ptr<gpu::MetalFrameData>> compute_frame_raw(ImageView<const std::uint8_t> raw_left,
                                                                   ImageView<const std::uint8_t> raw_right,
                                                                   ImageU8* rect_left = nullptr, ImageU8* rect_right = nullptr);

    [[nodiscard]] const StereoTimings& last_timings() const { return timings_; }

private:
    struct Impl;
    explicit MetalStereo(std::unique_ptr<Impl> impl);
    Result<void> encode_and_run(bool from_raw, bool make_points, ImageU8* rect_left, ImageU8* rect_right);
    [[nodiscard]] depth::StereoResult read_result() const;
    std::unique_ptr<Impl> impl_;
    StereoTimings timings_;
};

}  // namespace einstar::depth_metal
