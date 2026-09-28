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
    // Accepted physical diameters (mm) and tolerance. EXStar ships 6 mm markers; 3 mm ones exist but
    // are close to the size of laser speckle dots, so they must be enabled explicitly.
    std::vector<double> diameters{6.0};
    double diameter_tolerance = 0.35;  // relative
    double max_size_mismatch = 0.35;   // relative difference of the left/right apparent sizes after depth scaling
    double prior_tolerance_px = 6.0;   // |d - d_prior| when a disparity prior is available
    // Only accept markers whose surroundings have depth (a prior was given but has no value for this
    // marker): stickers sit on scanned surfaces, and unverifiable pairings are the main source of
    // phantom markers floating in front of the scene.
    bool require_prior = false;
};

class MarkerStereo {
public:
    MarkerStereo(const RigCalibration& rig, const calib::StereoRectification& rect, StereoParams params = {});

    // Rectified full-resolution coordinates of a raw-image point.
    [[nodiscard]] Vec2 rectify_left(const Vec2& raw) const;
    [[nodiscard]] Vec2 rectify_right(const Vec2& raw) const;

    // Expected full-resolution rectified disparity at a rectified-left marker of the given apparent
    // radius (px), e.g. from the dense speckle depth around it. 0 = cannot be checked (e.g. outside the
    // depth image); negative = checked, but there is no surface around the marker.
    using DisparityPrior = std::function<double(const Vec2& left_rect, double radius_px)>;

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
