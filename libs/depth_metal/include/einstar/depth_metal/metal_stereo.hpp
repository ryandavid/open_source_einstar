#pragma once

// Metal implementation of the stereo pipeline in libs/depth (same algorithm, same parameters).
// All buffers use shared storage, so results are read by the CPU without copies.

#include <array>
#include <memory>
#include <vector>
#include <optional>

#include "einstar/calib/rectify.hpp"
#include "einstar/core/error.hpp"
#include "einstar/depth/stereo.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/gpu/device_data.hpp"

namespace einstar::depth_metal {

// GPU marker blob search on the raw images (the per-pixel part of markers::detect_markers).
struct BlobParams {
    std::uint32_t threshold = 150;
    std::uint32_t min_diameter = 3, max_diameter = 60;
    float max_aspect = 4.0f;
    float min_fill = 0.5f;
    std::uint32_t border = 4;
    float max_axis_ratio = 3.0f;
    // Dark-ring pre-check on the blob's moment ellipse (a little more lenient than the CPU's test on
    // the fitted ellipse, which follows).
    float ring_scale = 1.4f;
    float ring_contrast = 0.3f;
    float ring_max_bright = 0.2f;
    std::uint32_t max_blobs = 1024;
};
struct BlobBox {
    std::uint32_t x0, y0, x1, y1, pixels, peak;
};
static_assert(sizeof(BlobBox) == 24);

struct FrameRequest {
    bool preview_textures = false;  // rectified pair as GPU textures (for display)
    bool preview_images = false;    // rectified pair copied to CPU images
    bool marker_blobs = false;      // needs set_blob_params()
};

struct FrameOutputs {
    std::shared_ptr<gpu::MetalFrameData> frame;
    gpu::Ref<MTL::Texture> preview_left, preview_right;  // R8Unorm, private storage
    ImageU8 rect_left, rect_right;
    std::array<std::vector<BlobBox>, 2> blobs;           // left, right candidates
    std::array<std::uint32_t, 2> blobs_found{};          // before the max_blobs cap
};

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

    // Full GPU path with extras. Raw images whose storage is an einstar::Image (page aligned) are used
    // by the GPU in place; nothing is copied.
    void set_blob_params(const BlobParams& params);
    Result<FrameOutputs> compute_frame(const ImageU8& raw_left, const ImageU8& raw_right, const FrameRequest& request);

    [[nodiscard]] const StereoTimings& last_timings() const { return timings_; }

private:
    struct Impl;
    explicit MetalStereo(std::unique_ptr<Impl> impl);
    Result<void> encode_and_run(bool from_raw, bool make_points, ImageU8* rect_left, ImageU8* rect_right,
                                const FrameRequest* request = nullptr, FrameOutputs* outputs = nullptr);
    [[nodiscard]] depth::StereoResult read_result() const;
    std::unique_ptr<Impl> impl_;
    StereoTimings timings_;
};

}  // namespace einstar::depth_metal
