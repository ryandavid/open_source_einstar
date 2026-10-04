#pragma once

// Raw IR pair -> rectified half-resolution depth frame (640x512, like EXStar's point image).

#include <memory>
#include <mutex>
#include <optional>

#include "einstar/calib/rectify.hpp"
#include "einstar/depth/stereo.hpp"
#include "einstar/depth_metal/metal_stereo.hpp"
#include "einstar/markers/detect.hpp"
#include "einstar/markers/stereo.hpp"
#include "einstar/track/frame.hpp"
#include "einstar/usb/stream.hpp"

namespace einstar::pipeline {

enum class StereoBackend { automatic, cpu, metal };

struct StereoFrontendParams {
    StereoBackend backend = StereoBackend::automatic;
    double min_depth_mm = 150.0;
    double max_depth_mm = 700.0;
    // Where both rectified images show the same columns (calib::RectificationOptions): near the middle
    // of the depths scanned (most lie between 240 and 450 mm), so the fewest columns are lost either way.
    double reference_depth_mm = 320.0;
    depth::RefineParams refine;
    depth::SpeckleParams speckle{100, 1.0f};
    bool detect_markers = true;
    bool gpu_previews = true;   // rectified pair as GPU textures (Metal backend)
    bool cpu_previews = false;  // rectified pair copied to CPU images as well (debugging tools)
    markers::DetectParams marker_detect;
    markers::StereoParams marker_stereo{.require_prior = true};
    // Fill the depth hole under each stereo marker from a plane fitted to the surrounding surface.
    bool fill_marker_holes = true;
    double marker_fill_max_rms_mm = 0.2;     // surroundings must be this flat (at 300 mm; scales with z^2)
    double marker_fill_max_offset_mm = 1.0;  // and the marker must lie on them
    float marker_fill_weight = 0.5f;
};

struct DepthOutput {
    track::DepthFrame frame;
    ImageU8 rectified_left;   // half resolution; CPU backend, or on request (cpu_previews)
    ImageU8 rectified_right;
    gpu::Ref<MTL::Texture> preview_left, preview_right;  // GPU backend: the same images as textures
    std::vector<markers::Marker3D> markers;  // also in frame.markers (same order)
    std::vector<Vec2> unmatched_left;       // rectified full-resolution centres of left detections without a match
    double stereo_ms = 0;
    double marker_ms = 0;
    int marker_candidates = 0;  // blobs that reached the sub-pixel fit (both images)
};

class StereoFrontend {
public:
    StereoFrontend(const RigCalibration& rig, StereoFrontendParams params = {});

    // `left_sensor` is which device sensor index is the calibration's left camera (0 or 1);
    // unknown until detected (see detect_sensor_order).
    [[nodiscard]] DepthOutput process(const ImageU8& raw_left, const ImageU8& raw_right) const;
    [[nodiscard]] std::optional<DepthOutput> process(const usb::FrameGroup& group) const;

    // Runs stereo both ways on one group and keeps the order that yields more valid depth.
    int detect_sensor_order(const usb::FrameGroup& group);
    [[nodiscard]] int left_sensor() const { return left_sensor_; }
    void set_left_sensor(int s) { left_sensor_ = s; }

    [[nodiscard]] const track::Intrinsics& depth_intrinsics() const { return depth_k_; }
    [[nodiscard]] bool using_gpu() const { return metal_ != nullptr; }
    [[nodiscard]] const calib::StereoRectification& rectification() const { return rect_; }
    [[nodiscard]] const markers::MarkerStereo& marker_stereo() const { return *marker_stereo_; }

private:
    // Sub-pixel ellipse fits of the GPU's blob candidates (left, right).
    struct FittedMarkers {
        std::vector<markers::Ellipse> left, right;
        int candidates = 0;
        double ms = 0;
    };
    [[nodiscard]] FittedMarkers fit_gpu_blobs(const ImageU8& raw_left, const ImageU8& raw_right,
                                              const std::array<std::vector<depth_metal::BlobBox>, 2>& blobs) const;
    // Markers from `fitted`, or detected on the CPU when null.
    void add_markers(const ImageU8& raw_left, const ImageU8& raw_right, DepthOutput& out, FittedMarkers* fitted = nullptr) const;

    StereoFrontendParams params_;
    calib::StereoRectification rect_;
    calib::RemapTable map_left_, map_right_;
    depth::StereoParams stereo_;
    track::Intrinsics depth_k_;
    int left_sensor_ = 0;
    std::unique_ptr<markers::MarkerStereo> marker_stereo_;
    std::unique_ptr<depth_metal::MetalStereo> metal_;
    mutable std::mutex metal_mutex_;
};

}  // namespace einstar::pipeline
