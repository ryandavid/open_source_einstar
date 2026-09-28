// einstar-cli: headless tools.
//   probe [--verbose]                 read-only hardware identification (first contact with a scanner)
//   sim-probe                         same, against the device emulator
//   calib <dir-with-CCF-files>        decode LeftCCF/RightCCF/TexCCF and print the rig
//   track-fixture <project.ir_E10_prj> [--start N] [--count N] [--skip K] [--record out.estr] [--quiet]
//                                     run the tracker on EXStar-recorded depth frames and compare poses;
//                                     --record writes a session (plus EXStar's poses as <out>.exstar_poses)
//   process <session.estr> [-o mesh.stl|ply|obj] [--voxel MM] [--no-optimize] [--smooth N]
//           [--stl reference.stl] [--reference-poses file]
//                                     the process step: optimise poses, re-fuse, mesh, export

#include <algorithm>
#include <charconv>
#include <fstream>
#include <map>
#include <cstring>
#include <cmath>
#include <optional>
#include <format>
#include <iomanip>
#include <print>
#include <span>
#include <string_view>
#include <vector>

#include <Eigen/Geometry>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/device/einstar_device.hpp"
#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/recon/process.hpp"
#include "einstar/session/session.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/track/tracker.hpp"
#include "einstar/track_metal/metal_icp.hpp"
#include "einstar/track_metal/metal_tsdf.hpp"

using namespace einstar;

namespace {

int usage() {
    std::println(stderr,
                 "usage: einstar-cli probe [--verbose] | sim-probe | calib <dir> |\n"
                 "       track-fixture <project.ir_E10_prj> [--start N] [--count N] [--skip K] [--stl ref.stl] [--cpu] [--mode geometry|hybrid|markers] [--global-markers] [--marker-confirm N] [--record out.estr] [--quiet] |\n"
                 "       process <session.estr> [-o mesh.stl|ply|obj] [--voxel MM] [--no-optimize] [--smooth N] [--stl reference.stl] [--reference-poses file]");
    return 2;
}

long arg_int(std::span<char*> args, std::string_view name, long def) {
    for (std::size_t i = 0; i + 1 < args.size(); ++i)
        if (name == args[i]) {
            long v = def;
            std::from_chars(args[i + 1], args[i + 1] + std::strlen(args[i + 1]), v);
            return v;
        }
    return def;
}

const char* arg_str(std::span<char*> args, std::string_view name) {
    for (std::size_t i = 0; i + 1 < args.size(); ++i)
        if (name == args[i]) return args[i + 1];
    return nullptr;
}

double arg_double(std::span<char*> args, std::string_view name, double def) {
    const char* v = arg_str(args, name);
    return v ? std::atof(v) : def;
}

bool has_flag(std::span<char*> args, std::string_view name) {
    return std::ranges::any_of(args, [&](const char* a) { return name == a; });
}

void print_device(device::EinstarDevice& dev) {
    const auto& info = dev.info();
    std::println("vendor      {}", info.vendor_name);
    std::println("product     {}  (vid {:04x} pid {:04x})", info.product_name, info.vendor_id, info.product_id);
    std::println("serial      {}", info.serial);
    std::println("firmware    {}", info.firmware);
    for (int i = 0; i < info.sensor_count; ++i) {
        const auto& s = info.sensors[static_cast<std::size_t>(i)];
        std::println("sensor {}    {}x{} {}-bit {}  exposure {}..{}  gain {}..{}", i, s.width, s.height, s.pixel_bits,
                     s.color_mode ? "colour" : "mono", s.exposure_min, s.exposure_max, s.gain_min, s.gain_max);
    }
    if (auto t = dev.temperature_c()) std::println("temperature {:.2f} C", *t);
    if (auto blob = dev.read_flash(0, calib::kFlashBlobSize)) {
        auto cal = calib::decode_flash_blob(*blob);
        if (cal) {
            std::println("calibration {} (baseline {:.3f} mm)", cal->calibration_time, cal->rig().baseline_mm());
        } else {
            std::println("calibration blob read ({} bytes) but not decodable: {}", blob->size(), cal.error().message);
        }
    }
}

int probe(bool verbose) {
    auto devices = usb::enumerate_devices();
    if (!devices) {
        std::println(stderr, "enumeration failed: {}", devices.error().message);
        return 1;
    }
    if (devices->empty()) {
        std::println("no Shining3D devices (vid 3267) found");
        return 1;
    }
    for (const auto& d : *devices) std::println("found {:04x}:{:04x} at {}", d.vendor_id, d.product_id, d.path);
    auto transport = usb::open_libusb(devices->front());
    if (!transport) {
        std::println(stderr, "open failed: {}", transport.error().message);
        return 1;
    }
    device::ConnectOptions opts;
    opts.verbose_transcript = verbose;
    opts.transcript_sink = [](std::string_view line) { std::println("  {}", line); };
    opts.heartbeat_ms = 0;
    auto dev = device::EinstarDevice::connect(std::move(*transport), opts);
    if (!dev) {
        std::println(stderr, "connect failed: {}", dev.error().message);
        return 1;
    }
    print_device(**dev);
    return 0;
}

int sim_probe() {
    device::ConnectOptions opts;
    opts.heartbeat_ms = 0;
    auto dev = device::EinstarDevice::connect(std::make_unique<sim::SimTransport>(), opts);
    if (!dev) {
        std::println(stderr, "connect failed: {}", dev.error().message);
        return 1;
    }
    print_device(**dev);
    return 0;
}

int calib_cmd(const char* dir) {
    auto cal = calib::load_ccf_directory(dir);
    if (!cal) {
        std::println(stderr, "{}", cal.error().message);
        return 1;
    }
    for (const auto& [name, c] : {std::pair{"left", &cal->left}, std::pair{"right", &cal->right}, std::pair{"texture", &cal->texture}}) {
        const auto& m = c->model;
        std::println("{:8} f=({:.3f}, {:.3f}) c=({:.3f}, {:.3f}) k=({:.5f}, {:.5f}, {:.2e}, {:.2e}, {:.5f})", name, m.fx, m.fy,
                     m.cx, m.cy, m.dist[0], m.dist[1], m.dist[2], m.dist[3], m.dist[4]);
    }
    const auto rig = cal->rig();
    std::println("baseline {:.3f} mm, left->right rotation {:.3f} deg", rig.baseline_mm(),
                 rotation_angle(rig.T_right_left) * 180.0 / M_PI);
    return 0;
}

int track_fixture(const char* path, std::span<char*> args) {
    auto proj = fixtures::ExstarProject::open(path);
    if (!proj) {
        std::println(stderr, "{}", proj.error().message);
        return 1;
    }
    const long start = arg_int(args, "--start", 0);
    const long count = arg_int(args, "--count", static_cast<long>((*proj)->frame_count()));
    const long skip = std::max(1L, arg_int(args, "--skip", 1));
    const bool quiet = has_flag(args, "--quiet");
    const double frame_dt = 0.068;  // EXStar scan trigger period

    // Optional geometric check: distance of each (sampled) tracked frame to a reference mesh.
    std::optional<fixtures::Mesh> mesh;
    std::unique_ptr<fixtures::MeshDistance> mesh_dist;
    for (std::size_t a = 0; a + 1 < args.size(); ++a)
        if (std::string_view(args[a]) == "--stl") {
            auto m = fixtures::load_stl(args[a + 1]);
            if (!m) {
                std::println(stderr, "{}", m.error().message);
                return 1;
            }
            mesh = std::move(*m);
            mesh_dist = std::make_unique<fixtures::MeshDistance>(*mesh);
        }
    std::vector<double> fit_median;
    int fit_bad = 0, fit_checked = 0;

    std::unique_ptr<track::Volume> volume;
    if (!has_flag(args, "--cpu")) {
        if (auto ctx = gpu::Context::create())
            if (auto v = track_metal::MetalTsdfVolume::create(*ctx)) volume = std::move(*v);
    }
    std::unique_ptr<track_metal::MetalIcp> gpu_icp;
    if (!has_flag(args, "--cpu") && !has_flag(args, "--cpu-icp"))
        if (auto ctx = gpu::Context::create())
            if (auto g = track_metal::MetalIcp::create(*ctx)) gpu_icp = std::move(*g);
    track::TrackerParams tp;
    std::string mode = "hybrid";
    for (std::size_t a = 0; a + 1 < args.size(); ++a)
        if (std::string_view(args[a]) == "--mode") mode = args[a + 1];
    tp.mode = mode == "geometry" ? track::AlignMode::geometry : mode == "markers" ? track::AlignMode::markers : track::AlignMode::hybrid;
    tp.marker_map.min_observations = static_cast<int>(arg_int(args, "--marker-confirm", tp.marker_map.min_observations));
    tp.icp.degenerate_direction_ratio = arg_double(args, "--degen-ratio", tp.icp.degenerate_direction_ratio);
    std::println("volume: {}, icp: {}, mode: {}", volume ? "Metal" : "CPU", gpu_icp ? "Metal" : "CPU", mode);
    track::Tracker tracker(tp, std::move(volume));
    if (gpu_icp) tracker.set_icp_solver(gpu_icp->as_function());
    if (has_flag(args, "--global-markers")) {
        // EXStar's final marker map from the project header as a fixed global-marker map: the scan
        // then starts by locating itself on it, in EXStar's world frame.
        std::vector<markers::MapMarker> map;
        for (const auto& m : (*proj)->global_markers()) map.push_back({m.id, m.position, m.diameter > 1.5 ? m.diameter : 6.0, 1, true});
        std::println("global markers: {} fixed markers from the project header", map.size());
        tracker.set_marker_map(std::move(map));
    }
    std::unique_ptr<session::SessionWriter> recorder;
    std::ofstream exstar_poses;
    const char* record_path = arg_str(args, "--record");
    std::vector<double> t_err, r_err, ms, rpe_t, rpe_r, eigs;
    std::optional<SE3> prev_ours, prev_theirs;
    int gross = 0;
    int accepted = 0, lost = 0, reloc = 0, degenerate = 0, processed = 0, marker_frames = 0;
    const SE3* first_ref = nullptr;
    SE3 ref0, prev_ref;
    bool diverged = false;
    const long end = std::min<long>(start + count, static_cast<long>((*proj)->frame_count()));
    for (long i = start; i < end; i += skip) {
        auto f = (*proj)->read_frame(static_cast<std::size_t>(i));
        if (!f) {
            std::println(stderr, "frame {}: {}", i, f.error().message);
            return 1;
        }
        const track::Intrinsics k{f->depth.width(), f->depth.height(), f->intrinsics.fx, f->intrinsics.fy,
                                  f->intrinsics.cx, f->intrinsics.cy};
        auto frame = track::make_depth_frame(f->depth, k);
        frame.index = static_cast<std::uint64_t>(i);
        frame.timestamp_s = static_cast<double>(i - start) * frame_dt;
        for (const auto& m : f->markers) frame.markers.push_back({m.position, m.normal, m.diameter, -1});
        if (!first_ref) {
            ref0 = f->T_world_camera;
            first_ref = &ref0;
            tracker.set_initial_pose(ref0);
        }
        if (first_ref && processed > 0) {
            const SE3 step = prev_ref.inverse() * f->T_world_camera;
            if (translation_norm(step) > 20.0)
                std::println("frame {:5} EXStar pose jump {:.1f} mm / {:.1f} deg (discontinuity in the recording)", i,
                             translation_norm(step), rotation_angle(step) * 180.0 / M_PI);
        }
        prev_ref = f->T_world_camera;
        const auto r = tracker.process(frame);
        ++processed;
        if (record_path && !recorder) {
            session::SessionHeader h;
            h.depth_intrinsics = k;
            h.baseline_mm = (*proj)->baseline_mm();
            h.description = std::string("replay of ") + path;
            auto w = session::SessionWriter::create(record_path, h);
            if (!w) {
                std::println(stderr, "{}", w.error().message);
                return 1;
            }
            recorder = std::move(*w);
            if (has_flag(args, "--global-markers")) recorder->write_global_markers(tracker.marker_map().markers());
            exstar_poses.open(std::string(record_path) + ".exstar_poses");
        }
        if (recorder) {
            session::FrameRecord rec;
            rec.index = static_cast<std::uint64_t>(i);
            rec.timestamp_s = frame.timestamp_s;
            rec.flags = (r.accepted ? session::frame_accepted : 0u) | (r.degenerate ? session::frame_degenerate : 0u) |
                        (r.relocalized ? session::frame_relocalized : 0u) | (r.marker_pose ? session::frame_marker_pose : 0u) |
                        (r.integrated ? session::frame_integrated : 0u);
            rec.T_world_camera = r.T_world_camera;
            std::map<int, int> ids(r.marker_ids.begin(), r.marker_ids.end());
            for (std::size_t m = 0; m < frame.markers.size(); ++m) {
                const auto& mk = frame.markers[m];
                const auto it = ids.find(static_cast<int>(m));
                rec.markers.push_back({mk.position, mk.normal, mk.diameter, it == ids.end() ? -1 : it->second, mk.left_rect, mk.right_rect});
            }
            session::capture_depth(frame, rec.depth, rec.confidence);
            recorder->write(std::move(rec));
            const auto& M = f->T_world_camera.matrix();
            exstar_poses << i;
            for (int rr = 0; rr < 3; ++rr)
                for (int cc = 0; cc < 4; ++cc) exstar_poses << ' ' << std::setprecision(12) << M(rr, cc);
            exstar_poses << '\n';
        }
        if (r.accepted && !diverged) {
            const SE3 d = f->T_world_camera.inverse() * r.T_world_camera;
            if (translation_norm(d) > 3.0) {
                diverged = true;
                std::println("frame {:5} first divergence from EXStar: {:.1f} mm {:.2f} deg (rms {:.3f} inl {:.2f} cov {:.2f})", i,
                             translation_norm(d), rotation_angle(d) * 180.0 / M_PI, r.icp.rms_mm, r.icp.inlier_ratio,
                             r.icp.coverage);
            }
        }
        ms.push_back(r.ms);
        if (r.icp.converged) eigs.push_back(r.icp.min_eigenvalue_ratio);
        if (r.marker_pose) ++marker_frames;
        if (r.accepted) {
            if (prev_ours && prev_theirs) {
                const SE3 ours = prev_ours->inverse() * r.T_world_camera;
                const SE3 theirs = prev_theirs->inverse() * f->T_world_camera;
                if (translation_norm(theirs) < 20.0) {  // skip EXStar's recording discontinuities
                    const SE3 e = theirs.inverse() * ours;
                    rpe_t.push_back(translation_norm(e));
                    rpe_r.push_back(rotation_angle(e) * 180.0 / M_PI);
                }
            }
            prev_ours = r.T_world_camera;
            prev_theirs = f->T_world_camera;
            if (mesh_dist && accepted % 10 == 0) {
                std::vector<float> e;
                int far = 0;
                const Eigen::Matrix4f T = r.T_world_camera.matrix().cast<float>();
                for (const auto& p : fixtures::unproject(*f, 8)) {
                    if (auto d = mesh_dist->distance((T * p.homogeneous()).head<3>(), 2.0f)) e.push_back(*d);
                    else ++far;
                }
                if (!e.empty()) {
                    std::ranges::sort(e);
                    const double med = e[e.size() / 2];
                    const double far_frac = static_cast<double>(far) / static_cast<double>(e.size() + static_cast<std::size_t>(far));
                    fit_median.push_back(med);
                    ++fit_checked;
                    if (med > 0.5 || far_frac > 0.1) {
                        ++fit_bad;
                        if (!quiet) std::println("frame {:5} geometric misfit: median {:.2f} mm, {:.0f}% beyond 2 mm", i, med, 100 * far_frac);
                    }
                }
            }
            if (translation_norm(f->T_world_camera.inverse() * r.T_world_camera) > 20.0) {
                if (gross == 0)
                    std::println("frame {:5} first gross error {:.1f} mm (state reloc={} rms {:.3f} inl {:.2f} cov {:.2f} eig {:.1e})", i,
                                 translation_norm(f->T_world_camera.inverse() * r.T_world_camera), r.relocalized, r.icp.rms_mm,
                                 r.icp.inlier_ratio, r.icp.coverage, r.icp.min_eigenvalue_ratio);
                ++gross;
            }
            ++accepted;
            if (r.relocalized) ++reloc;
            if (r.degenerate) ++degenerate;
            const SE3 diff = f->T_world_camera.inverse() * r.T_world_camera;
            t_err.push_back(translation_norm(diff));
            r_err.push_back(rotation_angle(diff) * 180.0 / M_PI);
        } else {
            ++lost;
            prev_ours.reset();
        }
        const long trace_from = arg_int(args, "--trace-from", -1), trace_to = arg_int(args, "--trace-to", -1);
        if (i >= trace_from && i <= trace_to) {
            const SE3 d = f->T_world_camera.inverse() * r.T_world_camera;
            std::println("trace {:5} {:10} err {:6.2f} mm {:5.2f} deg  rms {:.3f} inl {:.2f} cov {:.2f} eig {:.1e} markers {}/{} "
                         "mrms {:.3f} mpose {} {}", i,
                         r.accepted ? (r.degenerate ? "ok-degen" : "ok") : "not-acc", translation_norm(d),
                         rotation_angle(d) * 180.0 / M_PI, r.icp.rms_mm, r.icp.inlier_ratio, r.icp.coverage,
                         r.icp.min_eigenvalue_ratio, r.markers_matched, r.markers_seen, r.icp.marker_rms_mm, r.marker_pose, r.reason);
        }
        if (!quiet && (!r.accepted || r.relocalized || processed % 100 == 0)) {
            std::println("frame {:5} {:6} rms {:.3f} inl {:.2f} cov {:.2f} eig {:.1e}  {:5.1f} ms  bricks {}  markers {}/{}  {}", i,
                         r.accepted ? (r.relocalized ? "reloc" : "ok") : "LOST", r.icp.rms_mm, r.icp.inlier_ratio,
                         r.icp.coverage, r.icp.min_eigenvalue_ratio, r.ms, tracker.volume().brick_count(), r.markers_matched,
                         r.markers_seen, r.reason);
        }
    }
    auto pct = [](std::vector<double> v, double p) {
        if (v.empty()) return 0.0;
        std::ranges::sort(v);
        return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))];
    };
    std::println("\nframes {} (skip {}), accepted {} ({:.1f}%), lost {}, relocalised {}, degenerate {}, marker-posed {}", processed, skip,
                 accepted, 100.0 * accepted / std::max(1, processed), lost, reloc, degenerate, marker_frames);
    std::println("marker map: {} markers", tracker.marker_map().size());
    std::println("pose vs EXStar: translation median {:.2f} mm p95 {:.2f} mm, rotation median {:.3f} deg p95 {:.3f} deg",
                 pct(t_err, 0.5), pct(t_err, 0.95), pct(r_err, 0.5), pct(r_err, 0.95));
    std::println("relative pose error per frame: translation median {:.3f} mm p95 {:.3f} mm, rotation median {:.4f} deg p95 {:.4f} deg",
                 pct(rpe_t, 0.5), pct(rpe_t, 0.95), pct(rpe_r, 0.5), pct(rpe_r, 0.95));
    std::println("pose differs from EXStar by >20 mm: {} frames (includes equally valid poses on symmetric surfaces)", gross);
    if (mesh_dist)
        std::println("geometric fit to reference mesh ({} sampled frames): median of medians {:.3f} mm, misfit frames {} ({:.1f}%)",
                     fit_checked, pct(fit_median, 0.5), fit_bad, 100.0 * fit_bad / std::max(1, fit_checked));
    std::println("icp eigen ratio percentiles: p1 {:.1e} p5 {:.1e} p10 {:.1e} p25 {:.1e} p50 {:.1e}", pct(eigs, 0.01), pct(eigs, 0.05),
                 pct(eigs, 0.10), pct(eigs, 0.25), pct(eigs, 0.5));
    std::println("time per frame: median {:.1f} ms p95 {:.1f} ms; model bricks {}", pct(ms, 0.5), pct(ms, 0.95),
                 tracker.volume().brick_count());
    if (recorder) {
        recorder->close();
        std::println("recorded {} frames ({:.1f} MB) to {}", recorder->frames_written(), static_cast<double>(recorder->bytes_written()) / 1e6,
                     recorder->path());
    }
    return 0;
}

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::ranges::sort(v);
    return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))];
}

int process_cmd(const char* path, std::span<char*> args) {
    auto s = session::SessionReader::open(path);
    if (!s) {
        std::println(stderr, "{}", s.error().message);
        return 1;
    }
    std::println("session: {} frames, {}", (*s)->frame_count(), (*s)->header().description);
    if (has_flag(args, "--verbose")) log::set_level(log::Level::debug);
    recon::ProcessParams pp;
    pp.tsdf.voxel_mm = static_cast<float>(arg_double(args, "--voxel", pp.tsdf.voxel_mm));
    pp.tsdf.truncation_mm = 5.0f * pp.tsdf.voxel_mm;
    pp.optimize_poses = !has_flag(args, "--no-optimize");
    pp.smooth_iterations = static_cast<int>(arg_int(args, "--smooth", 0));
    // Tuning knobs (see recon::ProcessParams).
    pp.graph_iterations = static_cast<int>(arg_int(args, "--graph-iterations", pp.graph_iterations));
    pp.loop_min_eigen_ratio = arg_double(args, "--loop-eig", pp.loop_min_eigen_ratio);
    pp.chain_sigma_deg = arg_double(args, "--chain-deg", pp.chain_sigma_deg);
    pp.chain_sigma_mm = arg_double(args, "--chain-mm", pp.chain_sigma_mm);
    pp.fragment_frames = static_cast<int>(arg_int(args, "--fragment-frames", pp.fragment_frames));
    pp.use_markers = !has_flag(args, "--no-markers");
    pp.recover_lost_frames = !has_flag(args, "--no-recover");
    pp.simplify = !has_flag(args, "--no-simplify");
    pp.simplify_params.max_error_mm = arg_double(args, "--simplify-error", pp.simplify_params.max_error_mm);
    pp.simplify_params.target_ratio = arg_double(args, "--simplify-ratio", pp.simplify_params.target_ratio);
    pp.marker_sigma_mm = arg_double(args, "--marker-sigma", pp.marker_sigma_mm);
    std::string last_stage;
    pp.progress = [&](const std::string& stage, double) {
        if (stage != last_stage) {
            std::println("  {}...", stage);
            last_stage = stage;
        }
    };
    auto r = recon::process_session(**s, pp);
    if (!r) {
        std::println(stderr, "process failed: {}", r.error().message);
        return 1;
    }
    const auto& rep = r->report;
    std::println("frames {} in {} fragments; {} odometry and {} loop-closure edges ({} candidates, {} pruned), {} graph iterations; "
                 "{} marker landmarks ({} observations)",
                 rep.frames_used, rep.fragments, rep.odometry_edges, rep.loop_edges, rep.loop_candidates, rep.loop_edges_pruned,
                 rep.graph_iterations, rep.marker_landmarks, rep.marker_observations);
    std::println("pose corrections: median {:.2f} mm, max {:.2f} mm / {:.2f} deg", rep.median_correction_mm, rep.max_correction_mm,
                 rep.max_correction_deg);
    std::println("islands: {} unverified segments, {} excluded ({} frames); {} lost frames recovered", rep.islands, rep.islands_excluded,
                 rep.frames_excluded, rep.frames_recovered);
    std::println("mesh: {} vertices, {} triangles ({} small pieces removed; simplified from {} triangles, max error {:.3f} mm)", rep.vertices,
                 rep.triangles, rep.cleanup.removed_components, rep.simplified.triangles_before, rep.simplified.max_error_mm);
    std::string times;
    for (const auto& [stage, ms] : rep.stage_ms) times += std::format(" {} {:.1f} s,", stage, ms / 1000.0);
    std::println("time:{}", times);

    if (const char* ref = arg_str(args, "--reference-poses")) {
        std::ifstream f(ref);
        std::map<std::size_t, SE3> exstar;
        std::size_t idx;
        while (f >> idx) {
            SE3 T = SE3::Identity();
            for (int rr = 0; rr < 3; ++rr)
                for (int cc = 0; cc < 4; ++cc) f >> T.matrix()(rr, cc);
            exstar[idx] = T;
        }
        std::vector<double> live_t, live_r, opt_t, opt_r;
        int good_to_bad = 0, bad_to_good = 0, bad_both = 0;
        for (const auto& [i, T] : r->frame_poses) {
            const auto it = exstar.find((*s)->meta(i).index);
            if (it == exstar.end()) continue;
            const SE3 el = it->second.inverse() * (*s)->meta(i).T_world_camera;
            const SE3 eo = it->second.inverse() * T;
            const bool lb = translation_norm(el) > 3.0, ob = translation_norm(eo) > 3.0;
            good_to_bad += !lb && ob;
            if (has_flag(args, "--verbose") && translation_norm((*s)->meta(i).T_world_camera.inverse() * T) > 5.0 && (*s)->meta(i).index % 10 == 0)
                std::println("  frame {}: live vs EXStar {:.1f} mm, processed vs EXStar {:.1f} mm, flags {:#x}", (*s)->meta(i).index,
                             translation_norm(el), translation_norm(eo), (*s)->meta(i).flags);
            bad_to_good += lb && !ob;
            bad_both += lb && ob;
            live_t.push_back(translation_norm(el));
            live_r.push_back(rotation_angle(el) * 180 / M_PI);
            opt_t.push_back(translation_norm(eo));
            opt_r.push_back(rotation_angle(eo) * 180 / M_PI);
        }
        std::println("pose vs EXStar ({} frames): live median {:.2f} mm p95 {:.2f} mm ({:.3f} / {:.3f} deg) -> processed median {:.2f} mm "
                     "p95 {:.2f} mm ({:.3f} / {:.3f} deg)",
                     opt_t.size(), percentile(live_t, 0.5), percentile(live_t, 0.95), percentile(live_r, 0.5), percentile(live_r, 0.95),
                     percentile(opt_t, 0.5), percentile(opt_t, 0.95), percentile(opt_r, 0.5), percentile(opt_r, 0.95));
        std::println("frames > 3 mm from EXStar: live only {}, processed only {}, both {}", bad_to_good, good_to_bad, bad_both);
        std::vector<double> rec_err;
        for (const auto& [i, T] : r->frame_poses) {
            if ((*s)->meta(i).accepted()) continue;
            const auto it = exstar.find((*s)->meta(i).index);
            if (it != exstar.end()) rec_err.push_back(translation_norm(it->second.inverse() * T));
        }
        if (!rec_err.empty())
            std::println("recovered frames vs EXStar ({}): median {:.2f} mm p95 {:.2f} mm, {} beyond 3 mm", rec_err.size(), percentile(rec_err, 0.5),
                         percentile(rec_err, 0.95), std::ranges::count_if(rec_err, [](double e) { return e > 3.0; }));
        // The process step fixes only the first frame, so the whole scan may differ from EXStar's by a
        // rigid transform; compare after the best rigid alignment of the camera centres.
        auto aligned = [&](bool processed) {
            std::vector<Vec3> a, b;
            std::vector<SE3> ours, theirs;
            for (const auto& [i, T] : r->frame_poses) {
                const auto it = exstar.find((*s)->meta(i).index);
                if (it == exstar.end()) continue;
                const SE3 P = processed ? T : (*s)->meta(i).T_world_camera;
                if (translation_norm(it->second.inverse() * P) > 3.0) continue;  // gross (symmetric) cases
                a.push_back(P.translation());
                b.push_back(it->second.translation());
                ours.push_back(P);
                theirs.push_back(it->second);
            }
            if (a.size() < 3) return std::pair{0.0, 0.0};
            Eigen::Matrix3Xd A(3, static_cast<Eigen::Index>(a.size())), B(3, static_cast<Eigen::Index>(b.size()));
            for (std::size_t n = 0; n < a.size(); ++n) A.col(static_cast<Eigen::Index>(n)) = a[n], B.col(static_cast<Eigen::Index>(n)) = b[n];
            SE3 G = SE3::Identity();
            G.matrix() = Eigen::umeyama(A, B, false);
            std::vector<double> et;
            for (std::size_t n = 0; n < ours.size(); ++n) et.push_back(translation_norm(theirs[n].inverse() * G * ours[n]));
            return std::pair{percentile(et, 0.5), percentile(et, 0.95)};
        };
        const auto [lm, l95] = aligned(false);
        const auto [pm, p95] = aligned(true);
        std::println("after rigid alignment to EXStar (frames within 3 mm): live median {:.2f} mm p95 {:.2f} mm -> processed median {:.2f} mm p95 {:.2f} mm",
                     lm, l95, pm, p95);
        if (has_flag(args, "--verbose")) {
            // Marker consistency: spread of each identified marker's world positions under each pose set.
            for (const char* which : {"EXStar", "live", "processed"}) {
                std::map<int, std::vector<Vec3>> pts;
                for (const auto& [i, T] : r->frame_poses) {
                    const auto it = exstar.find((*s)->meta(i).index);
                    if (it == exstar.end()) continue;
                    const SE3 P = std::string_view(which) == "EXStar" ? it->second : std::string_view(which) == "live" ? (*s)->meta(i).T_world_camera : T;
                    for (const auto& m : (*s)->meta(i).markers)
                        if (m.map_id >= 0) pts[m.map_id].push_back(P * m.position);
                }
                std::vector<double> spread;
                for (const auto& [id, v] : pts) {
                    if (v.size() < 5) continue;
                    Vec3 c = Vec3::Zero();
                    for (const auto& p : v) c += p;
                    c /= static_cast<double>(v.size());
                    for (const auto& p : v) spread.push_back((p - c).norm());
                }
                std::println("  marker spread under {} poses: median {:.3f} mm p95 {:.3f} mm", which, percentile(spread, 0.5), percentile(spread, 0.95));
            }
        }
    }
    if (const char* ref = arg_str(args, "--stl")) {
        auto m = fixtures::load_stl(ref);
        if (!m) {
            std::println(stderr, "{}", m.error().message);
            return 1;
        }
        // Accuracy: our surface -> reference. Completeness: reference -> our surface.
        const fixtures::MeshDistance to_ref(*m);
        fixtures::Mesh ours;
        ours.vertices.reserve(r->mesh.triangles.size() * 3);
        for (const auto& t : r->mesh.triangles)
            for (const auto v : t) ours.vertices.push_back(r->mesh.vertices[v]);
        const fixtures::MeshDistance to_ours(ours);
        std::vector<double> acc;
        int far = 0;
        const std::size_t step = std::max<std::size_t>(1, r->mesh.vertices.size() / 200000);
        for (std::size_t v = 0; v < r->mesh.vertices.size(); v += step) {
            if (auto d = to_ref.distance(r->mesh.vertices[v], 3.0f)) acc.push_back(*d);
            else ++far;
        }
        std::vector<double> comp;
        const std::size_t rstep = std::max<std::size_t>(1, m->vertices.size() / 200000);
        for (std::size_t v = 0; v < m->vertices.size(); v += rstep) comp.push_back(to_ours.distance(m->vertices[v], 3.0f).value_or(3.0f));
        const auto within = [](const std::vector<double>& d, double t) {
            return 100.0 * static_cast<double>(std::ranges::count_if(d, [&](double x) { return x <= t; })) / static_cast<double>(std::max<std::size_t>(1, d.size()));
        };
        std::println("vs reference mesh: accuracy median {:.3f} mm p90 {:.3f} mm p95 {:.3f} mm ({:.1f}% of our surface beyond 3 mm: not in the reference); "
                     "completeness {:.1f}% of the reference within 0.5 mm, {:.1f}% within 1 mm",
                     percentile(acc, 0.5), percentile(acc, 0.9), percentile(acc, 0.95), 100.0 * far / static_cast<double>(far + acc.size()),
                     within(comp, 0.5), within(comp, 1.0));
    }
    if (const char* outp = arg_str(args, "-o")) {
        if (auto w = recon::save_mesh(r->mesh, outp); !w) {
            std::println(stderr, "{}", w.error().message);
            return 1;
        }
        std::println("wrote {}", outp);
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string_view cmd = argv[1];
    std::span<char*> rest(argv + 2, static_cast<std::size_t>(argc - 2));
    if (cmd == "probe") return probe(has_flag(rest, "--verbose"));
    if (cmd == "sim-probe") return sim_probe();
    if (cmd == "calib" && argc >= 3) return calib_cmd(argv[2]);
    if (cmd == "track-fixture" && argc >= 3) return track_fixture(argv[2], rest);
    if (cmd == "process" && argc >= 3) return process_cmd(argv[2], std::span<char*>(argv + 3, static_cast<std::size_t>(argc - 3)));
    return usage();
}
