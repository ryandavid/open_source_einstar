#include "einstar/recon/process.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <numeric>
#include <set>

#include <tbb/parallel_for.h>
#include <tbb/task_group.h>

#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/optim/pose_graph.hpp"
#include "einstar/track_metal/metal_tsdf.hpp"

namespace einstar::recon {
namespace {

// A recorded frame as used for registration and fusion: depth edges removed, weights as configured.
track::DepthFrame frame_of(session::FrameRecord& rec, const track::Intrinsics& k, const ProcessParams& params) {
    if (params.edge_filter) track::filter_depth_edges(rec.depth, *params.edge_filter);
    auto f = rec.depth_frame(k);
    if (params.grazing_filter) track::filter_grazing(f, *params.grazing_filter);
    if (params.grazing_weight)
        for (int y = 0; y < f.points.height(); ++y)
            for (int x = 0; x < f.points.width(); ++x) {
                const Vec3f& n = f.normals(x, y);
                if (n.squaredNorm() > 0) f.weights(x, y) *= std::max(0.05f, -n.dot(f.points(x, y).normalized()));
            }
    return f;
}


struct Fragment {
    std::vector<std::size_t> frames;  // session indices
    std::size_t anchor = 0;           // the middle frame: registrations constrain its pose
    SE3 T_world_anchor = SE3::Identity();  // live pose of the anchor
    Cloud cloud;                           // anchor frame
    Vec3 center = Vec3::Zero();            // anchor frame
    double radius = 0;
    ImageF32 anchor_depth;                 // the anchor frame's depth at half resolution (free-space checks)
    track::Intrinsics anchor_k;
};

// Fraction of `pts` (in fragment b's anchor frame) that lie in space b's anchor camera saw as empty
// (in front of its observed surface), among those it saw at all. Wrong alignments that a dominant
// plane makes look good put the rest of the geometry into observed free space.
double free_space_violation(const std::vector<Vec3f>& pts, const Fragment& b, float tol_mm, int& tested) {
    const auto& k = b.anchor_k;
    const auto& D = b.anchor_depth;
    int viol = 0, seen = 0;
    const std::size_t step = std::max<std::size_t>(1, pts.size() / 3000);
    for (std::size_t i = 0; i < pts.size(); i += step) {
        const Vec3f& p = pts[i];
        if (p.z() <= 1.0f) continue;
        const int u = static_cast<int>(std::lround(k.fx * p.x() / p.z() + k.cx));
        const int v = static_cast<int>(std::lround(k.fy * p.y() / p.z() + k.cy));
        if (u < 0 || v < 0 || u >= D.width() || v >= D.height()) continue;
        const float d = D(u, v);
        if (d <= 0) continue;
        if (p.z() < d - tol_mm) ++viol;
        else if (p.z() < d + tol_mm) ++seen;
    }
    tested = viol + seen;
    return tested > 0 ? static_cast<double>(viol) / tested : 0.0;
}

// Symmetric free-space check of fragments a and b under the relative pose T_a_b (b -> a).
double pair_violation(const Fragment& a, const Fragment& b, const SE3& T_a_b, float tol_mm) {
    const Eigen::Matrix3f R = T_a_b.linear().cast<float>();
    const Vec3f t = T_a_b.translation().cast<float>();
    std::vector<Vec3f> b_in_a, a_in_b;
    b_in_a.reserve(b.cloud.size());
    for (const auto& p : b.cloud.points) b_in_a.push_back(R * p + t);
    const Eigen::Matrix3f Ri = R.transpose();
    for (const auto& p : a.cloud.points) a_in_b.push_back(Ri * (p - t));
    int n1 = 0, n2 = 0;
    const double v1 = free_space_violation(b_in_a, a, tol_mm, n1);
    const double v2 = free_space_violation(a_in_b, b, tol_mm, n2);
    return std::max(n1 >= 50 ? v1 : 0.0, n2 >= 50 ? v2 : 0.0);
}

struct FrameMarkers {
    std::vector<std::pair<int, Vec3>> markers;  // (live map id, camera-frame position)
    std::vector<std::pair<Vec3, double>> shape;  // (camera-frame normal, diameter), as `markers`
};

bool cancelled(const ProcessParams& p) { return p.cancel && p.cancel->load(); }

void progress(const ProcessParams& p, const std::string& stage, double f) {
    if (p.progress) p.progress(stage, f);
}

Cloud frame_cloud(const track::DepthFrame& f, const SE3& T, int stride) {
    Cloud c;
    const Eigen::Matrix3f R = T.linear().cast<float>();
    const Vec3f t = T.translation().cast<float>();
    for (int y = 0; y < f.points.height(); y += stride)
        for (int x = 0; x < f.points.width(); x += stride) {
            const Vec3f& p = f.points(x, y);
            const Vec3f& n = f.normals(x, y);
            if (p.z() <= 0 || n.squaredNorm() < 0.5f) continue;
            c.points.push_back(R * p + t);
            c.normals.push_back(R * n);
        }
    return c;
}

Mat6 diagonal_information(double sigma_deg, double sigma_mm, double scale) {
    const double sr = sigma_deg * M_PI / 180.0;
    Mat6 I = Mat6::Zero();
    I.diagonal() << 1 / (sr * sr), 1 / (sr * sr), 1 / (sr * sr), 1 / (sigma_mm * sigma_mm), 1 / (sigma_mm * sigma_mm),
        1 / (sigma_mm * sigma_mm);
    return I * scale;
}

}  // namespace

Result<ProcessResult> process_session(const session::SessionReader& s, const ProcessParams& params) {
    ProcessResult out;
    auto& rep = out.report;
    const auto& k = s.header().depth_intrinsics;
    Stopwatch total, sw;

    // ---- 1. Fragments -------------------------------------------------------------------------
    std::vector<std::size_t> tracked;  // session indices of tracked frames, in order
    std::vector<Fragment> frags;
    {
        double last_t = -1e9;
        for (std::size_t i = 0; i < s.frame_count(); ++i) {
            const auto& m = s.meta(i);
            if (!m.accepted()) continue;
            const bool gap = m.timestamp_s - last_t > params.fragment_max_gap_s;
            if (frags.empty() || gap || static_cast<int>(frags.back().frames.size()) >= params.fragment_frames) frags.emplace_back();
            frags.back().frames.push_back(i);
            tracked.push_back(i);
            last_t = m.timestamp_s;
        }
    }
    if (frags.empty()) return make_error(Errc::invalid_argument, "the session has no tracked frames");
    rep.frames_used = static_cast<int>(tracked.size());
    rep.fragments = static_cast<int>(frags.size());
    for (const auto i : tracked) out.frame_poses[i] = s.meta(i).T_world_camera;
    std::map<std::size_t, FrameMarkers> frame_markers;
    std::mutex mutex;
    std::string read_error;
    // Fragment clouds at the current frame poses (rebuilt after each pose-graph solve, so drift
    // inside a fragment does not bias its registrations).
    auto build_fragments = [&](bool collect_markers) {
        std::atomic<int> built{0};
        tbb::parallel_for(std::size_t{0}, frags.size(), [&](std::size_t fi) {
            if (cancelled(params)) return;
            auto& fr = frags[fi];
            fr.anchor = fr.frames[fr.frames.size() / 2];
            fr.T_world_anchor = out.frame_poses.at(fr.anchor);
            const SE3 T_anchor_world = fr.T_world_anchor.inverse();
            Cloud acc;
            for (const auto i : fr.frames) {
                auto rec = s.read(i);
                if (!rec) {
                    std::lock_guard lock(mutex);
                    read_error = rec.error().message;
                    return;
                }
                if (i == fr.anchor && fr.anchor_depth.empty()) {
                    fr.anchor_depth = ImageF32(rec->depth.width() / 2, rec->depth.height() / 2, 0.0f);
                    for (int y = 0; y < fr.anchor_depth.height(); ++y)
                        for (int x = 0; x < fr.anchor_depth.width(); ++x) fr.anchor_depth(x, y) = rec->depth(2 * x, 2 * y);
                    fr.anchor_k = k.scaled(0.5);
                }
                auto c = frame_cloud(frame_of(*rec, k, params), T_anchor_world * out.frame_poses.at(i), params.cloud_stride_px);
                acc.points.insert(acc.points.end(), c.points.begin(), c.points.end());
                acc.normals.insert(acc.normals.end(), c.normals.begin(), c.normals.end());
                if (!collect_markers) continue;
                FrameMarkers fm;
                for (const auto& mk : rec->markers)
                    if (mk.map_id >= 0) {
                        fm.markers.emplace_back(mk.map_id, mk.position);
                        fm.shape.emplace_back(mk.normal, mk.diameter);
                    }
                if (!fm.markers.empty()) {
                    std::lock_guard lock(mutex);
                    frame_markers[i] = std::move(fm);
                }
            }
            fr.cloud = voxel_downsample(acc, params.cloud_voxel_mm);
            Vec3 c = Vec3::Zero();
            for (const auto& p : fr.cloud.points) c += p.cast<double>();
            fr.center = fr.cloud.empty() ? Vec3::Zero() : Vec3(c / static_cast<double>(fr.cloud.size()));
            fr.radius = 0;
            for (const auto& p : fr.cloud.points) fr.radius = std::max(fr.radius, (p.cast<double>() - fr.center).norm());
            progress(params, "Building fragments", static_cast<double>(++built) / static_cast<double>(frags.size()));
        });
    };
    build_fragments(true);
    if (!read_error.empty()) return make_error(Errc::io, read_error);
    if (cancelled(params)) return make_error(Errc::busy, "cancelled");
    rep.stage_ms["fragments"] = sw.elapsed_ms();

    // ---- 2. Pose graph ------------------------------------------------------------------------
    std::vector<std::pair<std::size_t, std::size_t>> verified;  // fragment pairs linked by a registration
    std::map<int, std::set<std::size_t>> landmark_frames;         // marker landmark -> observing frames
    for (int iter = 0; params.optimize_poses && iter < params.graph_iterations; ++iter) {
        landmark_frames.clear();
        sw.reset();
        if (iter > 0) {
            build_fragments(false);
            if (!read_error.empty()) return make_error(Errc::io, read_error);
            if (cancelled(params)) return make_error(Errc::busy, "cancelled");
            rep.odometry_edges = rep.loop_edges = rep.loop_candidates = rep.loop_edges_pruned = 0;
        }
        std::vector<CloudIndex> index;
        index.reserve(frags.size());
        for (const auto& f : frags) index.emplace_back(f.cloud);

        optim::PoseGraph graph;
        for (const auto i : tracked) graph.nodes[static_cast<int>(i)] = out.frame_poses[i];
        // Chain: consecutive tracked frames keep their live relative pose, loosely.
        const Mat6 chain_info = diagonal_information(params.chain_sigma_deg, params.chain_sigma_mm, 1.0);
        for (std::size_t n = 1; n < tracked.size(); ++n) {
            const auto& a = s.meta(tracked[n - 1]);
            const auto& b = s.meta(tracked[n]);
            optim::PoseEdge e;
            e.i = static_cast<int>(tracked[n - 1]);
            e.j = static_cast<int>(tracked[n]);
            e.T_i_j = a.T_world_camera.inverse() * b.T_world_camera;
            const bool gap = b.timestamp_s - a.timestamp_s > params.fragment_max_gap_s || (b.flags & session::frame_relocalized);
            e.information = gap ? Mat6(chain_info * params.chain_gap_weight) : chain_info;
            graph.edges.push_back(e);
        }

        auto accept = [&](const RegistrationResult& r, const SE3& guess, double max_mm, double max_deg, std::size_t fi, std::size_t fj) {
            const SE3 d = guess.inverse() * r.T_target_source;
            return r.converged && r.fitness >= params.loop_min_fitness && r.rms_mm <= params.loop_max_rms_mm &&
                   r.min_eigen_ratio >= params.loop_min_eigen_ratio && translation_norm(d) <= max_mm &&
                   rotation_angle(d) * 180.0 / M_PI <= max_deg &&
                   pair_violation(frags[fi], frags[fj], r.T_target_source, params.free_space_tolerance_mm) <= params.max_free_space_violation;
        };
        auto fragment_edge = [&](std::size_t i, std::size_t j, const RegistrationResult& r, bool loop) {
            optim::PoseEdge e;
            e.i = static_cast<int>(frags[i].anchor);
            e.j = static_cast<int>(frags[j].anchor);
            e.T_i_j = r.T_target_source;
            e.information = r.information;
            e.loop = loop;
            return e;
        };
        auto current = [&](std::size_t f) { return graph.nodes[static_cast<int>(frags[f].anchor)]; };

        // Consecutive fragments overlap by construction: registration refines their relative pose.
        progress(params, "Registering fragments", 0.0);
        std::vector<std::optional<optim::PoseEdge>> odo(frags.size() - 1);
        tbb::parallel_for(std::size_t{0}, odo.size(), [&](std::size_t i) {
            const SE3 guess = frags[i].T_world_anchor.inverse() * frags[i + 1].T_world_anchor;
            const auto r = register_point_to_plane(frags[i + 1].cloud, index[i], guess, params.registration);
            if (accept(r, guess, 5.0, 3.0, i, i + 1)) odo[i] = fragment_edge(i, i + 1, r, false);
        });
        for (auto& e : odo)
            if (e) {
                graph.edges.push_back(*e);
                ++rep.odometry_edges;
            }
        std::map<std::pair<int, int>, std::pair<std::size_t, std::size_t>> anchor_pair;  // anchors -> fragments
        for (std::size_t i = 0; i + 1 < frags.size(); ++i)
            anchor_pair[{static_cast<int>(frags[i].anchor), static_cast<int>(frags[i + 1].anchor)}] = {i, i + 1};

        // Markers: landmarks observed by every frame that identified them.
        if (params.use_markers && !frame_markers.empty()) {
            std::map<int, std::pair<Vec3, int>> world;
            for (const auto& [i, fm] : frame_markers)
                for (const auto& [id, p] : fm.markers) {
                    auto& w = world[id];
                    w.first += out.frame_poses[i] * p;
                    ++w.second;
                }
            for (const auto& [id, w] : world) graph.landmarks[id] = w.first / w.second;
            for (const auto& gm : s.global_markers()) {
                graph.landmarks[gm.id] = gm.position;
                graph.fixed_landmarks.insert(gm.id);
            }
            for (const auto& [i, fm] : frame_markers)
                for (const auto& [id, p] : fm.markers) {
                    // Ids from different live maps (e.g. after a reset) can collide: keep only
                    // observations consistent with their landmark.
                    if ((out.frame_poses[i] * p - graph.landmarks[id]).norm() > params.marker_consistency_mm) continue;
                    graph.observations.push_back({static_cast<int>(i), id, p, params.marker_sigma_mm});
                }
            std::set<int> used;
            for (const auto& o : graph.observations) used.insert(o.landmark);
            std::erase_if(graph.landmarks, [&](const auto& kv) { return !used.contains(kv.first); });
            std::erase_if(graph.fixed_landmarks, [&](int id) { return !used.contains(id); });
            rep.marker_landmarks = static_cast<int>(graph.landmarks.size());
            rep.marker_observations = static_cast<int>(graph.observations.size());
        }

        // Loop closures between non-consecutive fragments that overlap at the current estimate. Each
        // round re-tests pairs that failed before: once nearer loops are closed, the poses of far
        // ones improve enough for registration to converge (drift grows around a loop).
        std::set<std::pair<std::size_t, std::size_t>> closed;
        std::map<std::pair<std::size_t, std::size_t>, SE3> tried;  // relative pose when last attempted
        double t_reg = 0, t_solve = 0;
        optim::PoseGraphParams gp;
        gp.prune_chi = params.prune_chi;
        gp.fixed_node = static_cast<int>(tracked.front());
        for (int round = 0; round < params.loop_rounds; ++round) {
            // Each fragment tries only its nearest overlapping partners (a revisited area has many
            // redundant ones), and a pair that failed before is retried only once the estimate of its
            // relative pose has changed.
            std::vector<std::pair<std::size_t, std::size_t>> cands;
            std::set<std::pair<std::size_t, std::size_t>> picked;
            for (std::size_t i = 0; i < frags.size(); ++i) {
                std::vector<std::pair<double, std::size_t>> near;
                const Vec3 ci = current(i) * frags[i].center;
                for (std::size_t j = 0; j < frags.size(); ++j) {
                    if (j + 1 >= i && j <= i + 1) continue;  // itself and consecutive fragments
                    const double d = (ci - current(j) * frags[j].center).norm();
                    if (d <= 0.8 * (frags[i].radius + frags[j].radius)) near.emplace_back(d, j);
                }
                std::ranges::sort(near);
                if (static_cast<int>(near.size()) > params.loop_max_partners) near.resize(static_cast<std::size_t>(params.loop_max_partners));
                for (const auto& [d, j] : near) {
                    const auto key = std::minmax(i, j);
                    if (closed.contains(key) || !picked.insert(key).second) continue;
                    const SE3 rel = current(key.first).inverse() * current(key.second);
                    if (const auto it = tried.find(key); it != tried.end()) {
                        const SE3 moved = it->second.inverse() * rel;
                        if (translation_norm(moved) < 1.0 && rotation_angle(moved) < 0.2 * M_PI / 180.0) continue;
                    }
                    tried[key] = rel;
                    cands.push_back(key);
                }
            }
            rep.loop_candidates += static_cast<int>(cands.size());
            std::vector<std::optional<optim::PoseEdge>> found(cands.size());
            std::atomic<int> done{0};
            Stopwatch reg_sw;
            tbb::parallel_for(std::size_t{0}, cands.size(), [&](std::size_t c) {
                if (cancelled(params)) return;
                const auto [i, j] = cands[c];
                const SE3 guess = current(i).inverse() * current(j);
                // Quick overlap test on a sparse sample before the full registration.
                const auto& src = frags[j].cloud;
                const std::size_t step = std::max<std::size_t>(1, src.size() / 300);
                const Eigen::Matrix3f R = guess.linear().cast<float>();
                const Vec3f t = guess.translation().cast<float>();
                int hit = 0, tot = 0;
                for (std::size_t p = 0; p < src.size(); p += step, ++tot) hit += index[i].nearest(R * src.points[p] + t, 3.0f) >= 0;
                if (tot > 0 && static_cast<double>(hit) / tot >= params.loop_min_overlap) {
                    const auto r = register_point_to_plane(src, index[i], guess, params.registration);
                    if (accept(r, guess, params.loop_max_correction_mm, params.loop_max_correction_deg, i, j)) found[c] = fragment_edge(i, j, r, true);
                    log::debug("loop {}-{} (anchors {}-{}): overlap {:.2f} conv {} fit {:.2f} rms {:.3f} eig {:.1e} conflict {:.2f} corr {:.2f} mm {:.2f} deg -> {}", i, j,
                               frags[i].anchor, frags[j].anchor, static_cast<double>(hit) / tot, r.converged, r.fitness, r.rms_mm, r.min_eigen_ratio, r.conflict,
                               translation_norm(guess.inverse() * r.T_target_source), rotation_angle(guess.inverse() * r.T_target_source) * 180 / M_PI,
                               found[c].has_value());
                }
                progress(params, "Finding loop closures", static_cast<double>(++done) / static_cast<double>(cands.size()));
            });
            if (cancelled(params)) return make_error(Errc::busy, "cancelled");
            t_reg += reg_sw.elapsed_ms();
            int added = 0;
            for (std::size_t c = 0; c < found.size(); ++c)
                if (found[c]) {
                    graph.edges.push_back(*found[c]);
                    closed.insert(cands[c]);
                    anchor_pair[{found[c]->i, found[c]->j}] = cands[c];
                    ++added;
                }
            rep.loop_edges += added;
            if (added == 0 && round > 0) break;
            progress(params, "Optimising poses", 0.0);
            Stopwatch solve_sw;
            const auto pr = optim::optimize(graph, gp);
            t_solve += solve_sw.elapsed_ms();
            log::debug("process: solve {:.0f} ms ({} solves)", solve_sw.elapsed_ms(), pr.solves);
            rep.loop_edges_pruned += pr.pruned_edges;
            log::info("process: round {}: {} candidates, {} loop closures ({} pruned), cost {:.4g} -> {:.4g}", round, cands.size(), added,
                      pr.pruned_edges, pr.cost_before, pr.cost_after);
        }
        // Registrations that survived pruning link fragments (used to find unverified islands).
        verified.clear();
        for (const auto& e : graph.edges)
            if (auto it = anchor_pair.find({e.i, e.j}); it != anchor_pair.end()) verified.push_back(it->second);
        for (const auto& o : graph.observations) landmark_frames[o.landmark].insert(static_cast<std::size_t>(o.node));
        double change = 0;
        std::vector<std::pair<double, std::size_t>> moved;
        for (auto& [i, T] : out.frame_poses) {
            const SE3& n = graph.nodes[static_cast<int>(i)];
            const double c = translation_norm(T.inverse() * n) + 100.0 * rotation_angle(T.inverse() * n);
            change = std::max(change, c);
            moved.emplace_back(c, i);
            T = n;
        }
        if (log::level() <= log::Level::debug) {
            std::ranges::sort(moved, std::greater{});
            std::string top;
            for (std::size_t m = 0; m < std::min<std::size_t>(12, moved.size()); ++m)
                top += std::format(" {}({:.1f}{})", s.meta(moved[m].second).index, moved[m].first,
                                   (s.meta(moved[m].second).flags & session::frame_degenerate) ? " degen" : "");
            const auto big = std::ranges::count_if(moved, [](const auto& m) { return m.first > 2.0; });
            log::debug("process: {} frames moved > 2 mm; largest:{}", big, top);
        }
        rep.stage_ms["pose graph"] += sw.elapsed_ms();
        ++rep.graph_iterations;
        log::debug("process: iteration {}: loop registration {:.0f} ms, solves {:.0f} ms", iter, t_reg, t_solve);
        log::info("process: pose graph iteration {}: largest pose change {:.3f} mm (at a 100 mm lever)", iter, change);
        if (change < params.graph_tolerance_mm) break;
    }

    // Islands: groups of fragments linked to the rest only by live tracking across a relocalisation
    // (no verified registration, no shared marker). A wrong live relocalisation (e.g. onto the
    // symmetric counterpart of a surface) produces such an island; if it contradicts the main model
    // where they overlap it is left out of the fusion.
    if (params.optimize_poses && frags.size() > 1) {
        std::vector<std::size_t> parent(frags.size());
        std::iota(parent.begin(), parent.end(), std::size_t{0});
        auto find = [&](std::size_t x) {
            while (parent[x] != x) x = parent[x] = parent[parent[x]];
            return x;
        };
        auto unite = [&](std::size_t a, std::size_t b) { parent[find(a)] = find(b); };
        std::map<std::size_t, std::size_t> frag_of;
        for (std::size_t f = 0; f < frags.size(); ++f)
            for (const auto i : frags[f].frames) frag_of[i] = f;
        for (std::size_t f = 0; f + 1 < frags.size(); ++f) {
            const auto& a = s.meta(frags[f].frames.back());
            const auto& b = s.meta(frags[f + 1].frames.front());
            const bool continuous = b.timestamp_s - a.timestamp_s <= params.fragment_max_gap_s && !(b.flags & session::frame_relocalized);
            if (continuous) unite(f, f + 1);
        }
        for (const auto& [a, b] : verified) unite(a, b);
        for (const auto& [id, frames] : landmark_frames)
            for (const auto i : frames) unite(frag_of.at(i), frag_of.at(*frames.begin()));
        std::map<std::size_t, std::vector<std::size_t>> comps;
        for (std::size_t f = 0; f < frags.size(); ++f) comps[find(f)].push_back(f);
        std::size_t main = comps.begin()->first, main_frames = 0;
        for (const auto& [root, fs] : comps) {
            std::size_t n = 0;
            for (const auto f : fs) n += frags[f].frames.size();
            if (n > main_frames) main_frames = n, main = root;
        }
        rep.islands = static_cast<int>(comps.size()) - 1;
        if (rep.islands > 0) {
            for (const auto& [root, fs] : comps) {
                if (root == main) continue;
                // Free-space test against every main fragment it overlaps.
                double worst = 0;
                std::size_t n_frames = 0;
                for (const auto f : fs) {
                    n_frames += frags[f].frames.size();
                    const SE3& Tf = out.frame_poses.at(frags[f].anchor);
                    for (const auto m : comps[main]) {
                        const SE3& Tm = out.frame_poses.at(frags[m].anchor);
                        if ((Tf * frags[f].center - Tm * frags[m].center).norm() > frags[f].radius + frags[m].radius) continue;
                        worst = std::max(worst, pair_violation(frags[m], frags[f], Tm.inverse() * Tf, params.free_space_tolerance_mm));
                    }
                }
                const bool contradicts = worst > params.max_free_space_violation;
                log::info("process: island of {} frames ({} fragments): free-space violation {:.0f}% -> {}", n_frames, fs.size(), 100 * worst,
                          contradicts ? "excluded" : "kept");
                if (!contradicts) continue;
                ++rep.islands_excluded;
                rep.frames_excluded += static_cast<int>(n_frames);
                for (const auto f : fs)
                    for (const auto i : frags[f].frames) out.frame_poses.erase(i);
            }
        }
    }
    tracked.clear();
    for (const auto& [i, T] : out.frame_poses) tracked.push_back(i);


    std::vector<double> corr;
    for (const auto& [i, T] : out.frame_poses) {
        const SE3 d = s.meta(i).T_world_camera.inverse() * T;
        corr.push_back(translation_norm(d));
        rep.max_correction_mm = std::max(rep.max_correction_mm, translation_norm(d));
        rep.max_correction_deg = std::max(rep.max_correction_deg, rotation_angle(d) * 180.0 / M_PI);
    }
    std::ranges::nth_element(corr, corr.begin() + static_cast<std::ptrdiff_t>(corr.size() / 2));
    rep.median_correction_mm = corr[corr.size() / 2];

    // ---- Lost-frame recovery ---------------------------------------------------------------------
    // Frames live tracking rejected are re-tracked against the complete model: each run of lost
    // frames is walked inward from both ends, starting from the neighbouring pose.
    if (params.recover_lost_frames && !frags.empty()) {
        sw.reset();
        Cloud model;
        for (const auto& f : frags) {
            const auto it = out.frame_poses.find(f.anchor);
            if (it == out.frame_poses.end()) continue;  // excluded island
            const auto c = transformed(f.cloud, it->second);
            model.points.insert(model.points.end(), c.points.begin(), c.points.end());
            model.normals.insert(model.normals.end(), c.normals.begin(), c.normals.end());
        }
        model = voxel_downsample(model, params.cloud_voxel_mm);
        const CloudIndex model_index(model);
        // Runs of consecutive untracked frames with a tracked neighbour on at least one side.
        std::vector<std::vector<std::size_t>> runs;
        for (std::size_t i = 0; i < s.frame_count(); ++i) {
            if (out.frame_poses.contains(i) || s.meta(i).accepted()) continue;  // tracked (or excluded) frames
            if (runs.empty() || runs.back().back() + 1 != i) runs.emplace_back();
            runs.back().push_back(i);
        }
        RegistrationParams rp = params.registration;
        rp.start_distance_mm = params.recover_start_distance_mm;
        rp.iterations = 40;
        std::mutex mtx;
        std::atomic<int> recovered{0}, done{0};
        tbb::parallel_for(std::size_t{0}, runs.size(), [&](std::size_t ri) {
            if (cancelled(params)) return;
            const auto& run = runs[ri];
            auto pose_of = [&](std::size_t i) -> std::optional<SE3> {
                std::lock_guard lock(mtx);
                const auto it = out.frame_poses.find(i);
                return it == out.frame_poses.end() ? std::nullopt : std::optional<SE3>(it->second);
            };
            std::map<std::size_t, SE3> found;
            auto try_frame = [&](std::size_t i, const SE3& guess) -> bool {
                auto rec = s.read(i);
                if (!rec) return false;
                const auto cloud = voxel_downsample(frame_cloud(frame_of(*rec, k, params), SE3::Identity(), params.recover_stride_px), params.cloud_voxel_mm);
                if (cloud.size() < 200) return false;
                const auto r = register_point_to_plane(cloud, model_index, guess, rp);
                const SE3 d = guess.inverse() * r.T_target_source;
                if (!r.converged || r.fitness < params.recover_min_fitness || r.rms_mm > params.recover_max_rms_mm ||
                    r.min_eigen_ratio < params.recover_min_eigen_ratio || translation_norm(d) > params.recover_max_correction_mm ||
                    rotation_angle(d) * 180.0 / M_PI > params.recover_max_correction_deg)
                    return false;
                found[i] = r.T_target_source;
                return true;
            };
            // Forward from the frame before the run, backward from the frame after it.
            std::optional<SE3> fwd = run.front() > 0 ? pose_of(run.front() - 1) : std::nullopt;
            std::size_t a = 0;
            for (; fwd && a < run.size(); ++a) {
                if (!try_frame(run[a], *fwd)) break;
                fwd = found[run[a]];
            }
            std::optional<SE3> bwd = pose_of(run.back() + 1);
            for (std::size_t b = run.size(); bwd && b > a; --b) {
                if (!try_frame(run[b - 1], *bwd)) break;
                bwd = found[run[b - 1]];
            }
            std::lock_guard lock(mtx);
            for (const auto& [i, T] : found) out.frame_poses[i] = T;
            recovered += static_cast<int>(found.size());
            progress(params, "Recovering lost frames", static_cast<double>(++done) / static_cast<double>(runs.size()));
        });
        rep.frames_recovered = recovered;
        rep.stage_ms["recover"] = sw.elapsed_ms();
        log::info("process: recovered {} of {} lost frames", recovered.load(),
                  std::accumulate(runs.begin(), runs.end(), std::size_t{0}, [](std::size_t n, const auto& r) { return n + r.size(); }));
    }

    // ---- 3. Re-fusion ---------------------------------------------------------------------------
    sw.reset();
    std::unique_ptr<track::Volume> volume;
    track::TsdfParams tsdf_params = params.tsdf;
    tsdf_params.count_observations = tsdf_params.count_observations || params.extract.min_observations > 0;
    if (params.use_gpu)
        if (auto ctx = gpu::Context::create())
            if (auto v = track_metal::MetalTsdfVolume::create(*ctx, tsdf_params)) volume = std::move(*v);
    if (!volume) volume = std::make_unique<track::TsdfVolume>(tsdf_params);
    // Fuses every frame at its final pose, each first checked against `model` if given.
    auto fuse = [&](const ConsistencyModel* model, const std::string& stage) -> Result<void> {
        // Decoding (zstd, points, normals) and the model checks run in parallel batches, the next
        // batch while the current one is integrated; integration stays in order.
        const std::vector<std::pair<std::size_t, SE3>> todo(out.frame_poses.begin(), out.frame_poses.end());
        constexpr std::size_t kBatch = 32;
        struct Batch {
            std::vector<track::DepthFrame> frames = std::vector<track::DepthFrame>(kBatch);
            std::vector<float> weight = std::vector<float>(kBatch);
            std::vector<std::string> errors = std::vector<std::string>(kBatch);
            std::vector<ConsistencyStats> stats = std::vector<ConsistencyStats>(kBatch);
        };
        std::array<Batch, 2> batches;
        auto prepare = [&](Batch& bt, std::size_t b) {
            tbb::parallel_for(std::size_t{0}, std::min(kBatch, todo.size() - b), [&](std::size_t j) {
                bt.errors[j].clear();
                auto rec = s.read(todo[b + j].first);
                if (!rec) {
                    bt.errors[j] = rec.error().message;
                    return;
                }
                // Read applies the session's erases at the live pose; a frame live tracking lost has
                // none, so at the pose recovered for it.
                if (!rec->accepted()) s.apply_erasures(todo[b + j].first, *rec, todo[b + j].second);
                bt.frames[j] = frame_of(*rec, k, params);
                bt.weight[j] = (rec->flags & session::frame_degenerate) ? params.degenerate_weight : 1.0f;
                if (model) bt.stats[j] = model->filter(bt.frames[j], todo[b + j].second, *params.consistency);
            });
        };
        if (!todo.empty()) prepare(batches[0], 0);
        for (std::size_t b = 0, cur = 0; b < todo.size(); b += kBatch, cur ^= 1) {
            if (cancelled(params)) return make_error(Errc::busy, "cancelled");
            tbb::task_group next;
            if (b + kBatch < todo.size()) next.run([&, b, cur] { prepare(batches[cur ^ 1], b + kBatch); });
            const auto& bt = batches[cur];
            const std::size_t n = std::min(kBatch, todo.size() - b);
            for (std::size_t j = 0; j < n; ++j) {
                if (!bt.errors[j].empty()) {
                    next.wait();
                    return make_error(Errc::io, bt.errors[j]);
                }
                volume->integrate(bt.frames[j], todo[b + j].second, bt.weight[j]);
                if (model) rep.consistency += bt.stats[j];
            }
            next.wait();
            progress(params, stage, static_cast<double>(b + n) / static_cast<double>(todo.size()));
        }
        return {};
    };
    if (auto r = fuse(nullptr, "Fusing"); !r) return std::unexpected(r.error());
    rep.stage_ms["fusion"] = sw.elapsed_ms();
    if (params.consistency) {
        // The consensus model as it would be meshed; then every frame again, checked against it.
        // (A second round against the cleaner model changed nothing measurable.)
        sw.reset();
        progress(params, "Checking frames against the model", 0.0);
        auto model_mesh = extract_mesh(*volume, params.extract);
        remove_small_components(model_mesh, params.cleanup);
        const ConsistencyModel model(model_mesh);
        volume->clear();
        if (auto r = fuse(&model, "Checking frames against the model"); !r) return std::unexpected(r.error());
        const auto& c = rep.consistency;
        log::info("process: consistency: {} of {} depth pixels dropped ({} in front of the model, {} behind it, {} by normal), {} not seen by it",
                  c.rejected(), c.pixels, c.in_front, c.behind, c.normal, c.unseen);
        rep.stage_ms["consistency"] = sw.elapsed_ms();
    }
    if (const auto* mv = dynamic_cast<const track_metal::MetalTsdfVolume*>(volume.get()); mv && mv->pool_exhausted())
        log::warn("process: the GPU brick pool is full; parts of the model are missing (use a larger voxel size)");

    // ---- 4. Mesh -------------------------------------------------------------------------------
    sw.reset();
    progress(params, "Meshing", 0.0);
    out.mesh = extract_mesh(*volume, params.extract);
    rep.cleanup = remove_small_components(out.mesh, params.cleanup);
    if (params.marker_flatten) {
        // Each identified marker where its observations agree at the final poses (ids from different
        // live maps can collide: observations far from the mean are dropped, then it is recomputed).
        std::map<int, std::vector<std::tuple<Vec3, Vec3, double>>> obs;  // id -> (world position, normal, diameter)
        for (const auto& [i, fm] : frame_markers) {
            const auto it = out.frame_poses.find(i);
            if (it == out.frame_poses.end()) continue;
            for (std::size_t m = 0; m < fm.markers.size(); ++m)
                obs[fm.markers[m].first].emplace_back(it->second * fm.markers[m].second, it->second.linear() * fm.shape[m].first, fm.shape[m].second);
        }
        std::vector<MarkerDisc> discs;
        for (const auto& [id, o] : obs) {
            Vec3 mean = Vec3::Zero();
            for (const auto& [p, n, d] : o) mean += p;
            mean /= static_cast<double>(o.size());
            Vec3 pos = Vec3::Zero(), nrm = Vec3::Zero();
            std::vector<double> diam;
            for (const auto& [p, n, d] : o)
                if ((p - mean).norm() <= params.marker_consistency_mm) pos += p, nrm += n, diam.push_back(d);
            if (diam.size() < 3 || nrm.norm() < 1e-9) continue;
            std::ranges::nth_element(diam, diam.begin() + static_cast<std::ptrdiff_t>(diam.size() / 2));
            discs.push_back({(pos / static_cast<double>(diam.size())).cast<float>(), nrm.normalized().cast<float>(),
                             static_cast<float>(diam[diam.size() / 2] / 2)});
        }
        rep.markers_flattened = static_cast<int>(flatten_markers(out.mesh, discs, *params.marker_flatten));
    }
    rep.holes_filled = static_cast<int>(fill_small_holes(out.mesh, params.fill_holes_max_perimeter_mm));
    if (params.smooth_iterations > 0) taubin_smooth(out.mesh, params.smooth_iterations);
    if (params.simplify) {
        progress(params, "Simplifying", 0.0);
        rep.simplified = recon::simplify(out.mesh, params.simplify_params);
    }
    rep.vertices = out.mesh.vertices.size();
    rep.triangles = out.mesh.triangles.size();
    rep.stage_ms["mesh"] = sw.elapsed_ms();
    rep.stage_ms["total"] = total.elapsed_ms();
    progress(params, "Done", 1.0);
    return out;
}

}  // namespace einstar::recon
