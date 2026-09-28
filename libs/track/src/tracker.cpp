#include "einstar/track/tracker.hpp"

#include <cmath>
#include <format>

#include "einstar/core/timing.hpp"

namespace einstar::track {

Tracker::Tracker(TrackerParams params, std::unique_ptr<Volume> volume)
    : params_(params), volume_(volume ? std::move(volume) : std::make_unique<TsdfVolume>(params.tsdf)), map_(params.marker_map) {}

void Tracker::reset(bool keep_fixed_markers) {
    volume_->clear();
    if (keep_fixed_markers) {
        std::vector<markers::MapMarker> fixed;
        for (const auto& m : map_.markers())
            if (m.fixed) fixed.push_back(m);
        map_.set_markers(std::move(fixed));
    } else {
        map_.clear();
    }
    initial_pose_.reset();
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

std::optional<SE3> Tracker::global_candidate(const DepthFrame& frame, std::string& why) {
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
    if (!r) {
        why = "global reloc found no match";
        return std::nullopt;
    }
    if (!r->ambiguous) return r->T_model_frame;
    // Several distinct poses fit (symmetric part): the scanner is most likely still near where
    // tracking was lost; otherwise wait for a more distinctive view.
    const SE3* nearest = nullptr;
    double best = params_.ambiguous_reloc_max_mm;
    for (const auto& c : r->candidates) {
        const double d = (c.T_model_frame.translation() - lost_pose_.translation()).norm();
        if (d < best) best = d, nearest = &c.T_model_frame;
    }
    if (!nearest) {
        why = std::format("global reloc ambiguous ({} poses fit, none near the loss)", r->candidates.size());
        return std::nullopt;
    }
    return *nearest;
}

TrackResult Tracker::process(const DepthFrame& frame) {
    Stopwatch sw;
    TrackResult out;
    const double t = frame.timestamp_s;

    std::vector<Vec3> fm;
    std::vector<double> fd;
    const bool use_markers = params_.mode != AlignMode::geometry && frame.markers.size() >= 3;
    if (use_markers)
        for (const auto& m : frame.markers) {
            fm.push_back(m.position);
            fd.push_back(m.diameter);
        }
    out.markers_seen = static_cast<int>(frame.markers.size());

    if (state_ == TrackState::initializing) {
        SE3 start = initial_pose_.value_or(SE3::Identity());
        if (!map_.empty() && params_.mode != AlignMode::geometry) {
            // A (global) marker map already exists: the scan starts in its coordinate frame, so the
            // first frame must be located against it.
            std::optional<markers::PoseEstimate> p;
            if (use_markers) p = map_.relocalize(fm, reloc_seed_++);
            if (!p || static_cast<int>(p->inliers.size()) < params_.min_marker_inliers) {
                out.state = state_;
                out.reason = use_markers ? "markers do not match the global marker map" : "no markers in view to start on the global marker map";
                out.ms = sw.elapsed_ms();
                return out;
            }
            start = p->T_world_camera;
        }
        last_pose_ = prev_pose_ = start;
        if (params_.fuse_surface) volume_->integrate(frame, last_pose_);
        if (use_markers) {
            map_.update(fm, fd, last_pose_, {});
            map_.confirm_all();  // the first frame defines the map
            for (std::size_t i = 0; i < fm.size(); ++i)
                if (auto slot = map_.nearest(last_pose_ * fm[i], params_.marker_map.merge_radius_mm, true))
                    out.marker_ids.emplace_back(static_cast<int>(i), map_.markers()[static_cast<std::size_t>(*slot)].id);
        }
        last_time_ = prev_time_ = t;
        state_ = TrackState::tracking;
        out.state = state_;
        out.T_world_camera = last_pose_;
        out.accepted = true;
        out.integrated = params_.fuse_surface;
        out.marker_pose = use_markers;
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

    // Marker pose: associate with the prediction, or relocalise against the whole map.
    std::optional<markers::PoseEstimate> mpose;
    if (use_markers && !map_.empty()) {
        if (state_ != TrackState::lost) mpose = map_.track(fm, seeds.front());
        if (!mpose) mpose = map_.relocalize(fm, reloc_seed_++);
        if (mpose && static_cast<int>(mpose->inliers.size()) < params_.min_marker_inliers) mpose.reset();
        if (mpose) seeds.insert(seeds.begin(), mpose->T_world_camera);
    }

    IcpParams icp = params_.icp;
    if (mpose) {
        icp.marker_weight = params_.marker_weight;
        for (const auto& c : mpose->inliers) icp.markers.push_back({c.p_camera, c.q_world});
    }
    if (state_ == TrackState::lost && !mpose) {
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
            IcpParams fine = icp;
            fine.levels = 1;
            const IcpResult r2 = icp_(frame, model2, r.T_world_camera, r.T_world_camera, fine);
            if (r2.converged) r = r2;
        }
        const bool strict = (state_ == TrackState::lost || state_ == TrackState::confirming) && !mpose;
        auto verdict = check(r, last_pose_, state_ == TrackState::lost ? 0.5 : dt, strict);
        // Markers are an independent identity check: a pose they agree with is accepted even when the
        // surface overlap alone would be too weak (new areas, featureless or symmetric parts).
        const bool marker_redundant = mpose && static_cast<int>(icp.markers.size()) >= params_.min_marker_override_inliers;
        if (marker_redundant && r.converged && r.marker_rms_mm <= params_.max_marker_rms_mm) verdict.reset();
        if (params_.mode == AlignMode::markers && marker_redundant && (!r.converged || r.marker_rms_mm > params_.max_marker_rms_mm)) {
            // Markers-only mode: fall back to the marker pose when the surface disagrees.
            r.T_world_camera = mpose->T_world_camera;
            r.converged = true;
            r.marker_rms_mm = mpose->rms_mm;
            verdict.reset();
        }
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
        std::string why;
        if (auto seed = global_candidate(frame, why)) {
            const RaycastResult model = volume_->raycast(*seed, mk);
            const IcpResult r = icp_(frame, model, *seed, *seed, params_.icp);
            const auto verdict = check(r, *seed, 0.1, true);  // motion gate is meaningless here
            if (!verdict) best = r;
            else last_reason = "global reloc rejected: " + *verdict;
        } else {
            last_reason = why;
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
    out.marker_pose = mpose.has_value();
    if (mpose && (state_ == TrackState::lost || state_ == TrackState::confirming)) {
        // Marker constellations identify the location unambiguously: no confirmation window needed.
        out.relocalized = state_ == TrackState::lost;
        confirm_count_ = 0;
    } else if (state_ == TrackState::lost || state_ == TrackState::confirming) {
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
    if (params_.fuse_surface) {
        volume_->integrate(frame, out.T_world_camera, out.degenerate ? params_.degenerate_weight : 1.0f, out.degenerate);
        out.integrated = true;
    }
    if (use_markers) {
        auto at_pose = map_.track(fm, out.T_world_camera);
        std::vector<markers::Correspondence> matched;
        if (at_pose) {
            // Associate at the accepted pose (do not re-fit: the joint solution is the pose).
            for (const auto& c : at_pose->inliers)
                if ((out.T_world_camera * c.p_camera - c.q_world).norm() <= params_.marker_map.match_radius_mm) matched.push_back(c);
        }
        out.markers_matched = static_cast<int>(matched.size());
        map_.update(fm, fd, out.T_world_camera, matched);
        // Report ids after the update (new markers now have ids too).
        for (std::size_t i = 0; i < fm.size(); ++i)
            if (auto slot = map_.nearest(out.T_world_camera * fm[i], params_.marker_map.merge_radius_mm, true))
                out.marker_ids.emplace_back(static_cast<int>(i), map_.markers()[static_cast<std::size_t>(*slot)].id);
    }
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
