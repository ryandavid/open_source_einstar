// einstar-cli: headless tools.
//   probe [--verbose]                 read-only hardware identification (first contact with a scanner)
//   sim-probe                         same, against the device emulator
//   calib <dir-with-CCF-files>        decode LeftCCF/RightCCF/TexCCF and print the rig
//   track-fixture <project.ir_E10_prj> [--start N] [--count N] [--skip K] [--quiet]
//                                     run the tracker on EXStar-recorded depth frames and compare poses

#include <algorithm>
#include <charconv>
#include <cstring>
#include <cmath>
#include <optional>
#include <print>
#include <span>
#include <string_view>
#include <vector>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/device/einstar_device.hpp"
#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/track/tracker.hpp"
#include "einstar/track_metal/metal_icp.hpp"
#include "einstar/track_metal/metal_tsdf.hpp"

using namespace einstar;

namespace {

int usage() {
    std::println(stderr,
                 "usage: einstar-cli probe [--verbose] | sim-probe | calib <dir> |\n"
                 "       track-fixture <project.ir_E10_prj> [--start N] [--count N] [--skip K] [--stl ref.stl] [--cpu] [--mode geometry|hybrid|markers] [--quiet]");
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
    std::println("volume: {}, icp: {}, mode: {}", volume ? "Metal" : "CPU", gpu_icp ? "Metal" : "CPU", mode);
    track::Tracker tracker(tp, std::move(volume));
    if (gpu_icp) tracker.set_icp_solver(gpu_icp->as_function());
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
            std::println("trace {:5} {:10} err {:6.2f} mm {:5.2f} deg  rms {:.3f} inl {:.2f} cov {:.2f} eig {:.1e} {}", i,
                         r.accepted ? (r.degenerate ? "ok-degen" : "ok") : "not-acc", translation_norm(d),
                         rotation_angle(d) * 180.0 / M_PI, r.icp.rms_mm, r.icp.inlier_ratio, r.icp.coverage,
                         r.icp.min_eigenvalue_ratio, r.reason);
        }
        if (!quiet && (!r.accepted || processed % 100 == 0)) {
            std::println("frame {:5} {:6} rms {:.3f} inl {:.2f} cov {:.2f} eig {:.1e}  {:5.1f} ms  bricks {}  {}", i,
                         r.accepted ? (r.relocalized ? "reloc" : "ok") : "LOST", r.icp.rms_mm, r.icp.inlier_ratio,
                         r.icp.coverage, r.icp.min_eigenvalue_ratio, r.ms, tracker.volume().brick_count(), r.reason);
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
    return usage();
}
