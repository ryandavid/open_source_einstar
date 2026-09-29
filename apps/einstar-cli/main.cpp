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
#include <thread>
#include <chrono>
#include <fstream>
#include <map>
#include <cstring>
#include <cmath>
#include <optional>
#include <format>
#include <print>
#include <span>
#include <string_view>
#include <vector>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/device/einstar_device.hpp"
#include "einstar/eval/evaluation.hpp"
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
                 "       process <session.estr> [-o mesh.stl|ply|obj] [--voxel MM] [--no-optimize] [--smooth N] [--stl reference.stl] [--reference-poses file] |\n"
                 "       inspect <session.estr>");
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
                     i == 2 ? "colour" : "mono", s.exposure_min, s.exposure_max, s.gain_min, s.gain_max);
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

// Where a frame disagrees with the model: each source pixel (at `pose`) classified like the ICP
// association, written as a BMP (green inlier, red in front of the model, blue behind it, yellow
// normal mismatch, grey no model) with counts and a histogram of the signed misses.
void diagnose_frame(const track::DepthFrame& frame, const track::Volume& volume, const SE3& pose, const track::IcpParams& ip,
                    double model_scale, const std::string& out_bmp) {
    frame.ensure_cpu();
    const auto mk = frame.intrinsics.scaled(model_scale);
    const auto model = volume.raycast(pose, mk);
    model.ensure_cpu();
    const int W = frame.points.width(), H = frame.points.height();
    std::vector<std::uint8_t> rgb(static_cast<std::size_t>(W * H) * 3, 0);
    const float cos_max = std::cos(ip.max_normal_angle_deg * static_cast<float>(M_PI) / 180.0f);
    const Eigen::Matrix4f T = pose.matrix().cast<float>(), Tinv = pose.inverse().matrix().cast<float>();
    int n_in = 0, n_front = 0, n_behind = 0, n_normal = 0, n_off = 0;
    std::map<int, int> hist;  // signed depth difference (frame - model) in mm, outliers only
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const Vec3f& pc = frame.points(x, y);
            if (pc.z() <= 0) continue;
            std::uint8_t* px = &rgb[(static_cast<std::size_t>(H - 1 - y) * static_cast<std::size_t>(W) + static_cast<std::size_t>(x)) * 3];
            auto set = [&](std::uint8_t r, std::uint8_t g, std::uint8_t b) { px[0] = b; px[1] = g; px[2] = r; };  // BMP is BGR
            const Vec3f pw = (T * pc.homogeneous()).head<3>();
            const int u = static_cast<int>(std::lround(mk.fx * pc.x() / pc.z() + mk.cx));
            const int v = static_cast<int>(std::lround(mk.fy * pc.y() / pc.z() + mk.cy));
            if (u < 0 || v < 0 || u >= mk.width || v >= mk.height || !model.valid(u, v)) {
                ++n_off;
                set(90, 90, 90);
                continue;
            }
            const Vec3f& q = model.points(u, v);
            const Vec3f& nq = model.normals(u, v);
            const float r = nq.dot(pw - q);
            if (std::abs(r) > ip.max_distance_mm || (pw - q).norm() > 2 * ip.max_distance_mm) {
                const float dz = pc.z() - (Tinv * q.homogeneous()).z();
                ++hist[static_cast<int>(std::floor(std::clamp(dz, -20.0f, 20.0f) / 2.0f)) * 2];
                if (dz < 0) ++n_front, set(230, 40, 40);
                else ++n_behind, set(40, 80, 240);
                continue;
            }
            const Vec3f ns = pose.linear().cast<float>() * frame.normals(x, y);
            if (ns.squaredNorm() > 0 && ns.dot(nq) < cos_max) {
                ++n_normal;
                set(230, 210, 40);
                continue;
            }
            ++n_in;
            set(60, 200, 90);
        }
    const int on = n_in + n_front + n_behind + n_normal;
    std::println("diagnose: inliers {} ({:.0f}% of on-model), in front of model {}, behind {}, normal mismatch {}, off model {}", n_in,
                 100.0 * n_in / std::max(1, on), n_front, n_behind, n_normal, n_off);
    std::print("diagnose: outlier depth difference (frame - model, mm):");
    for (const auto& [b, c] : hist) std::print(" [{},{}):{}", b, b + 2, c);
    std::println("");
    std::ofstream f(out_bmp, std::ios::binary);
    const std::uint32_t row = static_cast<std::uint32_t>(W) * 3, size = 54 + row * static_cast<std::uint32_t>(H);
    auto u32 = [&](std::uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); };
    auto u16 = [&](std::uint16_t v) { f.write(reinterpret_cast<const char*>(&v), 2); };
    f.write("BM", 2);
    u32(size), u32(0), u32(54), u32(40), u32(static_cast<std::uint32_t>(W)), u32(static_cast<std::uint32_t>(H));
    u16(1), u16(24), u32(0), u32(row * static_cast<std::uint32_t>(H)), u32(2835), u32(2835), u32(0), u32(0);
    f.write(reinterpret_cast<const char*>(rgb.data()), static_cast<std::streamsize>(rgb.size()));  // W * 3 is a multiple of 4
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
    const bool realtime = has_flag(args, "--realtime");
    const auto replay_start = std::chrono::steady_clock::now();
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
    tp.degenerate_weight = static_cast<float>(arg_double(args, "--degen-weight", tp.degenerate_weight));
    tp.degenerate_eigen_ratio = arg_double(args, "--degen-flag", tp.degenerate_eigen_ratio);
    tp.deterministic_relocalisation = !has_flag(args, "--live-reloc");  // live: results used whenever ready
    tp.feature_model_min_interval_frames = static_cast<int>(arg_int(args, "--rebuild-interval", tp.feature_model_min_interval_frames));
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
    std::vector<int> episodes;  // lengths of runs of frames without an accepted pose
    double reloc_wait_ms = 0;
    // Recovery after the recording's large jumps (EXStar lost the scanner there too): frames until the
    // next accepted pose, and whether that pose agrees with EXStar's.
    long kidnap_from = -1;
    std::vector<double> recover_frames;
    int recover_wrong = 0;
    int run_lost = 0;
    std::optional<SE3> prev_ours, prev_theirs;
    int gross = 0;
    int gross_fit[4] = {};  // >20 mm frames by fit to the reference mesh: [ours*2 + exstar's]
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
            if (translation_norm(step) > 40.0 || rotation_angle(step) * 180.0 / M_PI > 8.0) kidnap_from = i;  // recording jumps
            if (translation_norm(step) > 20.0)
                std::println("frame {:5} EXStar pose jump {:.1f} mm / {:.1f} deg (discontinuity in the recording)", i,
                             translation_norm(step), rotation_angle(step) * 180.0 / M_PI);
        }
        prev_ref = f->T_world_camera;
        if (has_flag(args, "--oracle")) {
            // Model from EXStar's poses (no tracking): separates scene content from our registration errors.
            if (i == arg_int(args, "--diagnose-frame", -1)) {
                diagnose_frame(frame, tracker.volume(), f->T_world_camera, tp.icp, tp.model_scale,
                               arg_str(args, "--diagnose-out") ? arg_str(args, "--diagnose-out") : "diagnose.bmp");
                break;
            }
            tracker.volume().integrate(frame, f->T_world_camera);
            continue;
        }
        if (realtime) {
            // Frames at the scanner's rate, so background work sees live timing.
            const auto due = replay_start + std::chrono::microseconds(static_cast<long long>(frame.timestamp_s * 1e6));
            std::this_thread::sleep_until(due);
        }
        const auto r = tracker.process(frame);
        ++processed;
        if (const long probe = arg_int(args, "--probe-frame", -1); probe >= 0 && i % arg_int(args, "--probe-every", 100) == 0) {
            // How the model looks to a later frame (at EXStar's pose) as it grows: finds when a ghost appeared.
            auto pf = (*proj)->read_frame(static_cast<std::size_t>(probe));
            const auto pframe = track::make_depth_frame(pf->depth, k);
            std::print("probe at frame {:5}: ", i);
            diagnose_frame(pframe, tracker.volume(), pf->T_world_camera, tp.icp, tp.model_scale, "/dev/null");
        }
        if (i == arg_int(args, "--diagnose-frame", -1))
            diagnose_frame(frame, tracker.volume(), r.icp.T_world_camera, tp.icp, tp.model_scale,
                           arg_str(args, "--diagnose-out") ? arg_str(args, "--diagnose-out") : "diagnose.bmp");
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
            eval::write_pose(exstar_poses, static_cast<std::size_t>(i), f->T_world_camera);
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
        ms.push_back(r.ms - r.wait_ms);  // tracking-thread time; waits only exist to keep replays deterministic
        reloc_wait_ms += r.wait_ms;
        if (kidnap_from >= 0 && r.accepted) {
            recover_frames.push_back(static_cast<double>(i - kidnap_from));
            if (translation_norm(f->T_world_camera.inverse() * r.T_world_camera) > 20.0) ++recover_wrong;
            kidnap_from = -1;
        }
        if (!r.accepted) ++run_lost;
        else if (run_lost > 0) episodes.push_back(std::exchange(run_lost, 0));
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
            if (translation_norm(f->T_world_camera.inverse() * r.T_world_camera) > 20.0 && mesh_dist) {
                // Who is wrong? Fit of the frame to the reference mesh at our pose and at EXStar's.
                auto fit = [&](const SE3& pose) {
                    std::vector<float> e;
                    int far = 0;
                    const Eigen::Matrix4f T = pose.matrix().cast<float>();
                    for (const auto& p : fixtures::unproject(*f, 8)) {
                        if (auto d = mesh_dist->distance((T * p.homogeneous()).head<3>(), 2.0f)) e.push_back(*d);
                        else ++far;
                    }
                    if (e.empty()) return false;
                    std::ranges::sort(e);
                    return e[e.size() / 2] <= 0.5f && far <= static_cast<int>(e.size() + static_cast<std::size_t>(far)) / 10;
                };
                const bool ours = fit(r.T_world_camera), theirs = fit(f->T_world_camera);
                ++gross_fit[(ours ? 2 : 0) + (theirs ? 1 : 0)];
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
            const SE3 di = f->T_world_camera.inverse() * r.icp.T_world_camera;  // the ICP answer, even when rejected
            std::print("icp-err {:6.2f} mm ", translation_norm(di));
            std::println("trace {:5} {:10} err {:6.2f} mm {:5.2f} deg  rms {:.3f} inl {:.2f} cov {:.2f} eig {:.1e} markers {}/{} "
                         "mrms {:.3f} mpose {} {}", i,
                         r.accepted ? (r.degenerate ? "ok-degen" : "ok") : "not-acc", translation_norm(d),
                         rotation_angle(d) * 180.0 / M_PI, r.icp.rms_mm, r.icp.inlier_ratio, r.icp.coverage,
                         r.icp.min_eigenvalue_ratio, r.markers_matched, r.markers_seen, r.icp.marker_rms_mm, r.marker_pose, r.reason);
        }
        if (!quiet && (!r.accepted || r.relocalized || processed % 100 == 0)) {
            std::println("frame {:5} {:6} rms {:.3f} inl {:.2f} cov {:.2f} eig {:.1e}  {:5.1f} ms (+{:.0f} waiting)  bricks {}  markers {}/{}  {}", i,
                         r.accepted ? (r.relocalized ? "reloc" : "ok") : "LOST", r.icp.rms_mm, r.icp.inlier_ratio,
                         r.icp.coverage, r.icp.min_eigenvalue_ratio, r.ms - r.wait_ms, r.wait_ms, tracker.volume().brick_count(), r.markers_matched,
                         r.markers_seen, r.reason);
        }
    }
    const auto pct = [](const std::vector<double>& v, double p) { return eval::percentile(v, p); };
    std::println("\nframes {} (skip {}), accepted {} ({:.1f}%), lost {}, relocalised {}, degenerate {}, marker-posed {}", processed, skip,
                 accepted, 100.0 * accepted / std::max(1, processed), lost, reloc, degenerate, marker_frames);
    std::println("marker map: {} markers", tracker.marker_map().size());
    std::println("pose vs EXStar: translation median {:.2f} mm p95 {:.2f} mm, rotation median {:.3f} deg p95 {:.3f} deg",
                 pct(t_err, 0.5), pct(t_err, 0.95), pct(r_err, 0.5), pct(r_err, 0.95));
    std::println("relative pose error per frame: translation median {:.3f} mm p95 {:.3f} mm, rotation median {:.4f} deg p95 {:.4f} deg",
                 pct(rpe_t, 0.5), pct(rpe_t, 0.95), pct(rpe_r, 0.5), pct(rpe_r, 0.95));
    std::println("pose differs from EXStar by >20 mm: {} frames (includes equally valid poses on symmetric surfaces)", gross);
    if (mesh_dist)
        std::println(">20 mm frames against the reference mesh: both poses fit {}, only ours {}, only EXStar's {}, neither {}",
                     gross_fit[3], gross_fit[2], gross_fit[1], gross_fit[0]);
    if (mesh_dist)
        std::println("geometric fit to reference mesh ({} sampled frames): median of medians {:.3f} mm, misfit frames {} ({:.1f}%)",
                     fit_checked, pct(fit_median, 0.5), fit_bad, 100.0 * fit_bad / std::max(1, fit_checked));
    std::println("icp eigen ratio percentiles: p1 {:.1e} p5 {:.1e} p10 {:.1e} p25 {:.1e} p50 {:.1e}", pct(eigs, 0.01), pct(eigs, 0.05),
                 pct(eigs, 0.10), pct(eigs, 0.25), pct(eigs, 0.5));
    std::println("time per frame: median {:.1f} ms p95 {:.1f} ms max {:.1f} ms; model bricks {}; waited {:.0f} ms for the relocaliser",
                 pct(ms, 0.5), pct(ms, 0.95), ms.empty() ? 0.0 : std::ranges::max(ms), tracker.volume().brick_count(), reloc_wait_ms);
    if (run_lost > 0) episodes.push_back(run_lost);
    std::ranges::sort(episodes, std::greater{});
    std::print("lost episodes: {}, longest:", episodes.size());
    for (std::size_t e = 0; e < std::min<std::size_t>(5, episodes.size()); ++e) std::print(" {}", episodes[e]);
    std::println(" frames");
    std::println("after recording jumps: {} recoveries, frames to recover median {:.0f} p90 {:.0f} max {:.0f}, wrong pose {}",
                 recover_frames.size(), pct(recover_frames, 0.5), pct(recover_frames, 0.9),
                 recover_frames.empty() ? 0.0 : std::ranges::max(recover_frames), recover_wrong);
    if (recorder) {
        recorder->close();
        std::println("recorded {} frames ({:.1f} MB) to {}", recorder->frames_written(), static_cast<double>(recorder->bytes_written()) / 1e6,
                     recorder->path());
    }
    return 0;
}

// What a session file holds: the scanner, frames and their extras, drops, raw IR (first frame decoded).
int inspect_cmd(const char* path) {
    auto r = session::SessionReader::open(path);
    if (!r) {
        std::println(stderr, "{}", r.error().message);
        return 1;
    }
    const auto& s = **r;
    const auto& k = s.header().depth_intrinsics;
    std::println("{}: \"{}\", depth {}x{} f {:.1f}, rectified f {:.1f} baseline {:.2f} mm", path, s.header().description, k.width,
                 k.height, k.fx, s.header().rect_f, s.header().baseline_mm);
    if (const auto& d = s.device())
        std::println("scanner: {} {} serial {} firmware {}; calibration blob {} bytes; left fx {:.2f}", d->vendor, d->product, d->serial,
                     d->firmware, d->calibration_blob.size(), d->rig.left.fx);
    else
        std::println("scanner: not recorded");
    std::size_t accepted = 0, with_extras = 0;
    std::uint32_t exp_lo = ~0u, exp_hi = 0;
    for (std::size_t i = 0; i < s.frame_count(); ++i) {
        const auto& m = s.meta(i);
        if (m.accepted()) ++accepted;
        if (!m.extras) continue;
        ++with_extras;
        exp_lo = std::min(exp_lo, m.extras->capture.exposure[0]);
        exp_hi = std::max(exp_hi, m.extras->capture.exposure[0]);
    }
    const double span_s = s.frame_count() > 1 ? s.meta(s.frame_count() - 1).timestamp_s - s.meta(0).timestamp_s : 0.0;
    std::println("frames: {} ({} accepted) over {:.1f} s; {} with capture settings / tracking diagnostics{}", s.frame_count(), accepted, span_s,
                 with_extras, with_extras ? std::format(" (exposure {}..{})", exp_lo, exp_hi) : std::string{});
    std::map<std::string, int> reasons;
    for (const auto& d : s.dropped()) ++reasons[d.reason];
    std::print("not processed: {}", s.dropped().size());
    for (const auto& [why, n] : reasons) std::print(" | {} x{}", why, n);
    std::println("");
    if (s.raw_count() == 0) {
        std::println("raw IR: none");
        return 0;
    }
    auto first = s.read_raw(0);
    if (!first) {
        std::println(stderr, "raw IR: {}", first.error().message);
        return 1;
    }
    std::uint64_t pixels = 0;
    for (const auto& [sensor, img] : first->images) pixels += img.pixels().size();
    const double per_frame = static_cast<double>(s.raw_bytes()) / static_cast<double>(s.raw_count());
    std::println("raw IR: {} frames, {:.1f} MB ({:.2f}x compression, {:.1f} MB/s at 14.7 Hz); first: {} images",
                 s.raw_count(), static_cast<double>(s.raw_bytes()) / 1e6, static_cast<double>(pixels) / per_frame, per_frame * 14.7 / 1e6,
                 first->images.size());
    return 0;
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
        const auto reference = eval::read_poses(ref);
        const auto c = eval::compare_poses(**s, r->frame_poses, reference);
        std::println("pose vs EXStar ({} frames): live median {:.2f} mm p95 {:.2f} mm ({:.3f} / {:.3f} deg) -> processed median {:.2f} mm "
                     "p95 {:.2f} mm ({:.3f} / {:.3f} deg)",
                     c.frames, c.live_mm.median, c.live_mm.p95, c.live_deg.median, c.live_deg.p95, c.processed_mm.median, c.processed_mm.p95,
                     c.processed_deg.median, c.processed_deg.p95);
        std::println("frames > 3 mm from EXStar: live only {}, processed only {}, both {}", c.bad_live_only, c.bad_processed_only, c.bad_both);
        if (c.recovered > 0)
            std::println("recovered frames vs EXStar ({}): median {:.2f} mm p95 {:.2f} mm, {} beyond 3 mm", c.recovered, c.recovered_mm.median,
                         c.recovered_mm.p95, c.recovered_bad);
        std::println("after rigid alignment to EXStar (frames within 3 mm): live median {:.2f} mm p95 {:.2f} mm -> processed median {:.2f} mm p95 {:.2f} mm",
                     c.aligned_live_mm.median, c.aligned_live_mm.p95, c.aligned_processed_mm.median, c.aligned_processed_mm.p95);
        if (has_flag(args, "--verbose")) {
            auto show = [&](const char* name, const std::function<std::optional<SE3>(std::size_t)>& pose_of) {
                const auto sp = eval::marker_spread(**s, pose_of);
                std::println("  marker spread under {} poses: median {:.3f} mm p95 {:.3f} mm", name, sp.median, sp.p95);
            };
            show("EXStar", [&](std::size_t i) -> std::optional<SE3> {
                const auto it = reference.find((*s)->meta(i).index);
                return it == reference.end() || !r->frame_poses.contains(i) ? std::nullopt : std::optional<SE3>(it->second);
            });
            show("live", [&](std::size_t i) -> std::optional<SE3> {
                return r->frame_poses.contains(i) && (*s)->meta(i).accepted() ? std::optional<SE3>((*s)->meta(i).T_world_camera) : std::nullopt;
            });
            show("processed", [&](std::size_t i) -> std::optional<SE3> {
                const auto it = r->frame_poses.find(i);
                return it == r->frame_poses.end() ? std::nullopt : std::optional<SE3>(it->second);
            });
        }
    }
    if (const char* ref = arg_str(args, "--stl")) {
        auto m = fixtures::load_stl(ref);
        if (!m) {
            std::println(stderr, "{}", m.error().message);
            return 1;
        }
        const auto c = eval::compare_mesh(r->mesh, *m);
        std::println("vs reference mesh: accuracy median {:.3f} mm p90 {:.3f} mm p95 {:.3f} mm ({:.1f}% of our surface beyond 3 mm: not in the reference); "
                     "completeness {:.1f}% of the reference within 0.5 mm, {:.1f}% within 1 mm",
                     c.accuracy_mm.median, c.accuracy_p90_mm, c.accuracy_mm.p95, 100 * c.beyond_fraction, 100 * c.completeness_05,
                     100 * c.completeness_1);
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
    if (cmd == "inspect" && argc >= 3) return inspect_cmd(argv[2]);
    if (cmd == "process" && argc >= 3) return process_cmd(argv[2], std::span<char*>(argv + 3, static_cast<std::size_t>(argc - 3)));
    return usage();
}
