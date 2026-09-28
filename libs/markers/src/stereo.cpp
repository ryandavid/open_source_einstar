#include "einstar/markers/stereo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace einstar::markers {
namespace {

Vec2 to_rectified(const CameraModel& cam, const Mat3& R, const CameraModel& rect, const Vec2& raw) {
    const Vec2 n = calib::undistort_to_normalized(cam, raw);
    const Vec3 r = R * Vec3(n.x(), n.y(), 1.0);
    return {rect.fx * r.x() / r.z() + rect.cx, rect.fy * r.y() / r.z() + rect.cy};
}

}  // namespace

MarkerStereo::MarkerStereo(const RigCalibration& rig, const calib::StereoRectification& rect, StereoParams params)
    : rig_(rig), rect_(rect), params_(std::move(params)) {}

Vec2 MarkerStereo::rectify_left(const Vec2& raw) const { return to_rectified(rig_.left, rect_.R_left, rect_.rectified, raw); }
Vec2 MarkerStereo::rectify_right(const Vec2& raw) const { return to_rectified(rig_.right, rect_.R_right, rect_.rectified, raw); }

std::vector<Marker3D> MarkerStereo::reconstruct(const std::vector<Ellipse>& left, const std::vector<Ellipse>& right,
                                                const DisparityPrior& prior) const {
    const auto& g = rect_.geometry;
    std::vector<Vec2> rl(left.size()), rr(right.size());
    for (std::size_t i = 0; i < left.size(); ++i) rl[i] = rectify_left(left[i].center);
    for (std::size_t j = 0; j < right.size(); ++j) rr[j] = rectify_right(right[j].center);

    struct Candidate {
        int i, j;
        double cost;
        Marker3D m;
    };
    std::vector<Candidate> cands;
    std::vector<double> priors(left.size(), -1.0);
    if (prior)
        for (std::size_t i = 0; i < left.size(); ++i) priors[i] = prior(rl[i], left[i].a);
    for (std::size_t i = 0; i < left.size(); ++i)
        for (std::size_t j = 0; j < right.size(); ++j) {
            const double dy = std::abs(rl[i].y() - rr[j].y());
            if (dy > params_.max_row_error_px) continue;
            const double d = rl[i].x() - rr[j].x();
            if (d <= 0) continue;
            double prior_err = 0;
            {
                const double dp = priors[i];
                if (prior && params_.require_prior && dp < 0) {
                    // No surface around it: only acceptable where dense stereo cannot exist anyway (the
                    // right view of this point falls outside the rectified image).
                    const auto& rc = rect_.rectified;
                    const bool unverifiable = rr[j].x() < 0 || rr[j].x() >= rc.width || rr[j].y() < 0 || rr[j].y() >= rc.height;
                    if (!unverifiable) continue;
                }
                if (dp > 0) {
                    prior_err = std::abs(d - dp);
                    if (prior_err > params_.prior_tolerance_px) continue;
                }
            }
            const double z = g.depth_from_disparity(d);
            if (z < params_.min_depth_mm || z > params_.max_depth_mm) continue;
            // Physical size from the (geometric-mean) apparent diameter; foreshortening shrinks b only.
            const double dl = 2.0 * left[i].a * z / g.f, dr = 2.0 * right[j].a * z / g.f;
            if (std::abs(dl - dr) > params_.max_size_mismatch * std::max(dl, dr)) continue;
            const double diam = 0.5 * (dl + dr);
            double best_rel = std::numeric_limits<double>::max();
            for (const double nominal : params_.diameters) best_rel = std::min(best_rel, std::abs(diam - nominal) / nominal);
            if (!params_.diameters.empty() && best_rel > params_.diameter_tolerance) continue;
            Marker3D m;
            m.position = Vec3((rl[i].x() - g.cx) * z / g.f, (0.5 * (rl[i].y() + rr[j].y()) - g.cy) * z / g.f, z);
            m.diameter = diam;
            m.left_rect = rl[i];
            m.right_rect = rr[j];
            m.left_index = static_cast<int>(i);
            m.right_index = static_cast<int>(j);
            cands.push_back({static_cast<int>(i), static_cast<int>(j),
                             dy / params_.max_row_error_px + best_rel + prior_err / params_.prior_tolerance_px, m});
        }
    // Keep only correspondences that are unique on both sides after gating; with repeated patterns
    // along a scanline several geometrically valid pairings exist and none can be trusted.
    std::vector<int> per_left(left.size(), 0), per_right(right.size(), 0);
    for (const auto& c : cands) {
        ++per_left[static_cast<std::size_t>(c.i)];
        ++per_right[static_cast<std::size_t>(c.j)];
    }
    std::vector<Marker3D> out;
    for (const auto& c : cands)
        if (per_left[static_cast<std::size_t>(c.i)] == 1 && per_right[static_cast<std::size_t>(c.j)] == 1) out.push_back(c.m);
    return out;
}

}  // namespace einstar::markers
