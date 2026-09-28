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
    depth::RefineParams refine;
    depth::SpeckleParams speckle{100, 1.0f};
    bool detect_markers = true;
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
    ImageU8 rectified_left;   // half resolution, for previews
    ImageU8 rectified_right;
    std::vector<markers::Marker3D> markers;  // also in frame.markers (same order)
    std::vector<Vec2> unmatched_left;       // rectified full-resolution centres of left detections without a match
    double stereo_ms = 0;
    double marker_ms = 0;
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
    void add_markers(const ImageU8& raw_left, const ImageU8& raw_right, DepthOutput& out) const;

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
