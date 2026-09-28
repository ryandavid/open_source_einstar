#include "einstar/track/tracker.hpp"

#include <cmath>
#include <format>

#include "einstar/core/timing.hpp"

namespace einstar::track {

Tracker::Tracker(TrackerParams params, std::unique_ptr<Volume> volume)
    : params_(params), volume_(volume ? std::move(volume) : std::make_unique<TsdfVolume>(params.tsdf)) {}

void Tracker::reset() {
    volume_->clear();
    state_ = TrackState::initializing;
    have_velocity_ = false;
}

SE3 Tracker::predict(double t) const {
    if (!have_velocity_ || last_time_ <= prev_time_) return last_pose_;
    // Constant velocity in the camera frame, scaled by the elapsed time (capped so a long gap
    // does not extrapolate wildly).
    const SE3 delta = prev_pose_.inverse() * last_pose_;
    const double ratio = std::clamp((t - last_time_) / (last_time_ - prev_time_), 0.0, 3.0);
    Vec6 xi = ratio * se3_log(delta);
    // Motion along directions the geometry could not observe is not extrapolated: it was set by
    // the prediction itself, so extrapolating it would compound (e.g. spinning about a cylinder axis).
    if (degenerate_dirs_ > 0) {
        // The ICP basis lives in world-frame left-perturbation coordinates; convert the camera-frame
        // velocity to that frame via the adjoint (rotation part only matters for the directions).
        Vec6 scale;
        scale << 1, 1, 1, 0.01, 0.01, 0.01;
        const Mat3 R = last_pose_.linear();
        const Vec3 w_world = R * xi.tail<3>();
        const Vec3 v_world = R * xi.head<3>() - w_world.cross(last_pose_.translation());
        Vec6 xw;
        xw << v_world, w_world;
        Vec6 xs = twist_to_center(xw, degenerate_center_).cwiseQuotient(scale);
        for (int d = 0; d < degenerate_dirs_; ++d) {
            const Vec6 v = degenerate_basis_.col(d);
            xs -= v * v.dot(xs);
        }
        xw = twist_from_center(xs.cwiseProduct(scale), degenerate_center_);
        return se3_exp(xw) * last_pose_;
    }
    return last_pose_ * se3_exp(xi);
}

std::optional<std::string> Tracker::check(const IcpResult& r, const SE3& from, double dt, bool strict) const {
    if (!r.converged) return "icp did not converge";
    const double min_cov = strict ? params_.reloc_min_coverage : params_.min_coverage;
    const double min_inl = strict ? params_.reloc_min_inlier_ratio : params_.min_inlier_ratio;
    if (r.coverage < min_cov) return std::format("low overlap ({:.0f}%)", 100 * r.coverage);
    if (r.correspondences < params_.min_correspondences) return std::format("{} correspondences", r.correspondences);
    if (r.inlier_ratio < min_inl) return std::format("inconsistent ({:.0f}% inliers)", 100 * r.inlier_ratio);
    if (strict && r.min_eigenvalue_ratio < params_.reloc_min_eigen_ratio)
        return std::format("ambiguous geometry (eig {:.1e})", r.min_eigenvalue_ratio);
    if (r.rms_mm > params_.max_rms_mm) return std::format("residual {:.2f} mm", r.rms_mm);
    const SE3 motion = from.inverse() * r.T_world_camera;
    const double slack_dt = std::max(dt, 0.05) + 0.05;
    if (translation_norm(motion) > params_.max_speed_mm_s * slack_dt)
        return std::format("jump {:.1f} mm", translation_norm(motion));
    if (rotation_angle(motion) * 180.0 / M_PI > params_.max_rot_speed_deg_s * slack_dt)
        return std::format("rotation {:.1f} deg", rotation_angle(motion) * 180.0 / M_PI);
    return std::nullopt;
}

std::optional<SE3> Tracker::global_candidate(const DepthFrame& frame) {
    const std::size_t bricks = volume_->brick_count();
    if (feature_model_.empty() ||
        static_cast<double>(bricks) > static_cast<double>(feature_model_bricks_) * (1.0 + params_.feature_model_rebuild_growth)) {
        OrientedCloud model;
        for (const auto& sp : volume_->extract_points(0, 1.0f)) {
            model.points.push_back(sp.position);
            model.normals.push_back(sp.normal);
        }
        feature_model_ = FeatureModel(voxel_downsample(model, params_.global.voxel_mm), params_.global.feature_radius_mm);
        feature_model_bricks_ = bricks;
    }
    frame.ensure_cpu();
    OrientedCloud cloud;
    for (int v = 0; v < frame.points.height(); v += 2)
        for (int u = 0; u < frame.points.width(); u += 2) {
            const Vec3f& n = frame.normals(u, v);
            if (n.squaredNorm() == 0) continue;
            cloud.points.push_back(frame.points(u, v));
            cloud.normals.push_back(n);
        }
    const auto r = register_global(cloud, feature_model_, params_.global, reloc_seed_++);
    if (!r) return std::nullopt;
    return r->T_model_frame;
}

TrackResult Tracker::process(const DepthFrame& frame) {
    Stopwatch sw;
    TrackResult out;
    const double t = frame.timestamp_s;

    if (state_ == TrackState::initializing) {
        last_pose_ = prev_pose_ = initial_pose_.value_or(SE3::Identity());
        volume_->integrate(frame, last_pose_);
        last_time_ = prev_time_ = t;
        state_ = TrackState::tracking;
        out.state = state_;
        out.T_world_camera = last_pose_;
        out.accepted = out.integrated = true;
        out.ms = sw.elapsed_ms();
        return out;
    }

    const Intrinsics mk = frame.intrinsics.scaled(params_.model_scale);
    const double dt = t - last_time_;

    // Candidate starting poses: motion prediction first, then the last good pose.
    std::vector<SE3> seeds;
    if (state_ == TrackState::tracking || state_ == TrackState::confirming) {
        seeds.push_back(predict(t));
        if (have_velocity_) seeds.push_back(last_pose_);
    } else {
        seeds.push_back(last_pose_);
    }

    IcpParams icp = params_.icp;
    if (state_ == TrackState::lost) {
        // Relocalisation near the last good pose: wider gates, more iterations.
        icp.max_distance_mm *= 3.0f;
        icp.iterations = {20, 10, 6};
    }

    std::optional<IcpResult> best;
    std::string last_reason = "no seed";
    for (const SE3& seed : seeds) {
        const RaycastResult model = volume_->raycast(seed, mk);
        IcpResult r = icp_(frame, model, seed, seed, icp);
        // A second pass re-rendered from the refined pose tightens associations after large motion.
        if (r.converged && translation_norm(seed.inverse() * r.T_world_camera) > 2.0) {
            const RaycastResult model2 = volume_->raycast(r.T_world_camera, mk);
            IcpParams fine = params_.icp;
            fine.levels = 1;
            const IcpResult r2 = icp_(frame, model2, r.T_world_camera, r.T_world_camera, fine);
            if (r2.converged) r = r2;
        }
        const bool strict = state_ == TrackState::lost || state_ == TrackState::confirming;
        const auto verdict = check(r, last_pose_, state_ == TrackState::lost ? 0.5 : dt, strict);
        if (!verdict) {
            if (!best || r.rms_mm < best->rms_mm) best = r;
            break;
        }
        last_reason = *verdict;
        if (!best) out.icp = r;
    }

    // Local relocalisation failed: try registering the frame against the whole model.
    if (!best && state_ == TrackState::lost && params_.global_relocalization &&
        (lost_frames_ % std::max(1, params_.global_reloc_every)) == 0) {
        if (auto seed = global_candidate(frame)) {
            const RaycastResult model = volume_->raycast(*seed, mk);
            const IcpResult r = icp_(frame, model, *seed, *seed, params_.icp);
            const auto verdict = check(r, *seed, 0.1, true);  // motion gate is meaningless here
            if (!verdict) best = r;
            else last_reason = "global reloc rejected: " + *verdict;
        } else {
            last_reason = "global reloc found no match";
        }
    }

    if (!best) {
        if (state_ == TrackState::lost) ++lost_frames_;
        else lost_frames_ = 0;
        if (state_ == TrackState::tracking) lost_pose_ = last_pose_;
        if (state_ == TrackState::confirming) last_pose_ = lost_pose_;  // candidate failed: fall back
        state_ = TrackState::lost;
        confirm_count_ = 0;
        have_velocity_ = false;
        out.state = state_;
        out.T_world_camera = last_pose_;
        out.reason = last_reason;
        out.ms = sw.elapsed_ms();
        return out;
    }

    out.icp = *best;
    out.T_world_camera = best->T_world_camera;
    out.degenerate = best->min_eigenvalue_ratio < params_.degenerate_eigen_ratio || best->degenerate_directions > 0;
    degenerate_dirs_ = best->degenerate_directions;
    degenerate_basis_ = best->degenerate_basis;
    degenerate_center_ = best->center;

    lost_frames_ = 0;
    if (state_ == TrackState::lost || state_ == TrackState::confirming) {
        // Candidate relocalisation: follow it, but do not fuse until confirmed.
        if (state_ == TrackState::lost) confirm_count_ = 0;
        ++confirm_count_;
        if (confirm_count_ < params_.confirm_frames || out.degenerate) {
            state_ = TrackState::confirming;
            prev_pose_ = last_pose_;
            prev_time_ = last_time_;
            last_pose_ = out.T_world_camera;
            last_time_ = t;
            have_velocity_ = confirm_count_ > 1;
            out.state = state_;
            out.reason = std::format("confirming relocalisation {}/{}", confirm_count_, params_.confirm_frames);
            out.ms = sw.elapsed_ms();
            return out;
        }
        out.relocalized = true;
        confirm_count_ = 0;
    }
    out.accepted = true;
    // Degenerate frames (pose partly held by the motion prior) are fused with reduced weight so
    // they extend the model without reshaping what is already well observed.
    volume_->integrate(frame, out.T_world_camera, out.degenerate ? params_.degenerate_weight : 1.0f, out.degenerate);
    out.integrated = true;
    prev_pose_ = last_pose_;
    prev_time_ = last_time_;
    have_velocity_ = true;
    last_pose_ = out.T_world_camera;
    last_time_ = t;
    state_ = TrackState::tracking;
    out.state = state_;
    out.ms = sw.elapsed_ms();
    return out;
}

}  // namespace einstar::track
