#pragma once

// Stereo marker reconstruction: undistort + rectify ellipse centres, match along rectified rows,
// triangulate from disparity, gate by depth and physical diameter.

#include <functional>
#include <vector>

#include "einstar/calib/rectify.hpp"
#include "einstar/markers/detect.hpp"

namespace einstar::markers {

struct Marker3D {
    Vec3 position;        // rectified-left camera frame (the tracker's camera frame), mm
    double diameter = 0;  // estimated physical diameter, mm
    Vec2 left_rect, right_rect;  // rectified full-resolution centres (for bundle adjustment)
    int left_index = -1, right_index = -1;
};

struct StereoParams {
    double max_row_error_px = 0.8;  // |v_left - v_right| after rectification (EXStar: 0.35 px unrectified)
    double min_depth_mm = 150.0;
    double max_depth_mm = 700.0;
    // Accepted physical diameters (mm) and tolerance; EXStar ships 6 mm markers, 3 mm also exist.
    std::vector<double> diameters{6.0, 3.0};
    double diameter_tolerance = 0.35;  // relative
    double max_size_mismatch = 0.35;   // relative difference of the left/right apparent sizes after depth scaling
    double prior_tolerance_px = 6.0;   // |d - d_prior| when a disparity prior is available
};

class MarkerStereo {
public:
    MarkerStereo(const RigCalibration& rig, const calib::StereoRectification& rect, StereoParams params = {});

    // Rectified full-resolution coordinates of a raw-image point.
    [[nodiscard]] Vec2 rectify_left(const Vec2& raw) const;
    [[nodiscard]] Vec2 rectify_right(const Vec2& raw) const;

    // Expected full-resolution rectified disparity at a rectified-left point (e.g. from the dense
    // speckle depth around the marker), or a negative value if unknown.
    using DisparityPrior = std::function<double(const Vec2& left_rect)>;

    // Ambiguous correspondences (repeated patterns on the same scanline) are only resolved with a
    // prior; without one they are dropped rather than risk a wrong 3D marker.
    [[nodiscard]] std::vector<Marker3D> reconstruct(const std::vector<Ellipse>& left, const std::vector<Ellipse>& right,
                                                    const DisparityPrior& prior = {}) const;

    [[nodiscard]] const calib::StereoRectification& rectification() const { return rect_; }

private:
    RigCalibration rig_;
    calib::StereoRectification rect_;
    StereoParams params_;
};

}  // namespace einstar::markers
