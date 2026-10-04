#pragma once

// Which camera is "left". EXStar takes stream sensor 1 as the left IR camera and turns sensor 0's frames
// 180 degrees as the right one (usb::kLeftSensor). Until 2026-10-04 Einstar had it the other way round:
// sensor 0 left, sensor 1 turned. Both are valid stereo pairs (each is the other turned 180 degrees with
// left and right exchanged), but a calibration describes one of them: EXStar's 09-27 calibration fits our
// frames to 0.07-0.09 px rows in its own convention and is 1-7 px off in the other (docs/calibration.md 8.2).
//
// A calibration Einstar Calibration solved before the change (and wrote into a scanner) is in the old
// convention; swap_camera_convention converts it. check_camera_convention tells which convention a
// calibration is in by comparing it with one known to be EXStar's, e.g. the flash's factory section: the
// principal points of a scanner drift a few px over years, the swap moves them by 15-30 px.

#include <cstdint>
#include <span>
#include <string>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/camera.hpp"
#include "einstar/core/error.hpp"

namespace einstar::calib {

// The same rig described in the other convention (its own inverse): left and right exchanged, each
// camera turned 180 degrees about its optical axis (principal point mirrored through the image centre,
// tangential distortion negated); the texture camera's pose re-expressed from the new left camera.
[[nodiscard]] RigCalibration swap_camera_convention(const RigCalibration& rig);

enum class CameraConvention { exstar, swapped, unknown };

struct ConventionCheck {
    CameraConvention convention = CameraConvention::unknown;
    double as_is_px = 0;    // principal points' distance from the reference's (both cameras, summed)
    double swapped_px = 0;  // the same after swap_camera_convention
};

// `reference` must be in EXStar's convention (e.g. calib::decode_factory_section of the same scanner).
// Decided only with a clear margin (one distance at least 1.5x the other).
[[nodiscard]] ConventionCheck check_camera_convention(const RigCalibration& rig, const RigCalibration& reference);

// The scanner's calibration from its flash blob, in EXStar's convention: the quick calibration
// (decode_flash_blob), converted when its factory section shows it is in the old one.
struct FlashCalibration {
    DeviceCalibration calibration;
    RigCalibration rig;
    bool converted = false;
    ConventionCheck check;  // against the factory section (unknown when there is none)
};
[[nodiscard]] Result<FlashCalibration> read_flash_calibration(std::span<const std::uint8_t> blob);

}  // namespace einstar::calib
