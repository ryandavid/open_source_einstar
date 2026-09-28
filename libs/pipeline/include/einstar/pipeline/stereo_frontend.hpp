#pragma once

// Raw IR pair -> rectified half-resolution depth frame (640x512, like EXStar's point image).

#include <optional>

#include "einstar/calib/rectify.hpp"
#include "einstar/depth/stereo.hpp"
#include "einstar/track/frame.hpp"
#include "einstar/usb/stream.hpp"

namespace einstar::pipeline {

struct StereoFrontendParams {
    double min_depth_mm = 150.0;
    double max_depth_mm = 700.0;
    depth::RefineParams refine;
    depth::SpeckleParams speckle{100, 1.0f};
};

struct DepthOutput {
    track::DepthFrame frame;
    ImageU8 rectified_left;   // half resolution, for previews
    ImageU8 rectified_right;
    double stereo_ms = 0;
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
    [[nodiscard]] const calib::StereoRectification& rectification() const { return rect_; }

private:
    StereoFrontendParams params_;
    calib::StereoRectification rect_;
    calib::RemapTable map_left_, map_right_;
    depth::StereoParams stereo_;
    track::Intrinsics depth_k_;
    int left_sensor_ = 0;
};

}  // namespace einstar::pipeline
