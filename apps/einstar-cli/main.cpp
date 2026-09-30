// einstar-cli: headless tools.
//   probe [--verbose]                 read-only hardware identification (first contact with a scanner)
//   sim-probe                         same, against the device emulator
//   calib <dir-with-CCF-files>        decode LeftCCF/RightCCF/TexCCF and print the rig
//   calib-dump <dir>                  read the attached scanner's calibration and write it as CCF files
//                                     (byte-for-byte EXStar's cache files)
//   hw-ui [--countdown S] [--led-seconds S] [--button-seconds S]
//                                     indicator LEDs and buttons on a fixed timeline for someone at the scanner
//   hw-lights [--out DIR] [--countdown S] [--step-seconds S]
//                                     each light source on its own while streaming (timed, for someone watching)
//   markers-debug <ir.pgm> [--threshold T] [--ring-scale S] [--ring-contrast C] [--ring-bright F]
//                                     marker detection on a saved IR frame, with why each blob was rejected
//   stereo-debug <left.pgm> <right.pgm> <calibration dir>
//                                     depth and marker row offsets on a saved raw pair (as is and turned 180)
//   hw-capture [--out DIR] [--label NAME] [--groups N]
//                                     raw captures of the current pose under standard lighting conditions
//   rig-fit <calibration dir> <dir>...
//                                     how far raw pairs are from the calibration, and the rig correction that fits them
//   board-check <calibration dir> <dir>...
//                                     homography residual of the calibration board in each camera (model fit)
//   fixture-pack --out DIR [--mustang <Project1.ir_E10_prj>] [--stl mesh.stl] [--board <dir>]
//                                     pack the excerpts of real EXStar data the tests read
//                                     (tests/fixtures/external/README.md)
//   hw-test [--out DIR] [--seconds S] [--texture-seconds S] [--buttons S]
//                                     exercise the attached scanner: identification, flash, registers,
//                                     indicator LEDs, scan and texture streaming, depth and markers on
//                                     real frames, buttons (turns the projector and strobe on)
//   track-fixture <project.ir_E10_prj> [--start N] [--count N] [--skip K] [--record out.estr] [--quiet]
//                                     run the tracker on EXStar-recorded depth frames and compare poses;
//                                     --record writes a session (plus EXStar's poses as <out>.exstar_poses)
//   process <session.estr> [-o mesh.stl|ply|obj] [--voxel MM] [--no-optimize] [--smooth N]
//           [--stl reference.stl] [--reference-poses file]
//                                     the process step: optimise poses, re-fuse, mesh, export

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <charconv>
#include <thread>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <numeric>
#include <tuple>
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
#include "einstar/calib/rectify.hpp"
#include "einstar/calibrate/board.hpp"
#include "einstar/calibrate/captures.hpp"
#include "einstar/calibrate/plan.hpp"
#include "einstar/calibrate/solve.hpp"
#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/device/einstar_device.hpp"
#include "einstar/eval/evaluation.hpp"
#include "einstar/markers/detect.hpp"
#include "einstar/markers/stereo.hpp"
#include "einstar/optim/stereo_calibration.hpp"
#include "einstar/pipeline/stereo_frontend.hpp"
#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/fixtures/packaging.hpp"
#include "einstar/recon/process.hpp"
#include "einstar/session/session.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/track/tracker.hpp"
#include "einstar/track_metal/metal_icp.hpp"
#include "einstar/track_metal/metal_tsdf.hpp"
#include "einstar/usb/constants.hpp"

using namespace einstar;

namespace {

int usage() {
    std::println(stderr,
                 "usage: einstar-cli probe [--verbose] | sim-probe | calib <dir> | calib-dump <dir> |\n"
                 "       hw-test [--out DIR] [--seconds S] [--texture-seconds S] [--buttons S] |\n"
                 "       hw-ui [--countdown S] [--led-seconds S] [--button-seconds S] |\n"
                 "       hw-lights [--out DIR] [--countdown S] [--step-seconds S] |\n"
                 "       markers-debug <ir.pgm> [--threshold T] [--ring-scale S] [--ring-contrast C] [--ring-bright F] |\n"
                 "       stereo-debug <left.pgm> <right.pgm> <calibration dir> |\n"
                 "       hw-capture [--out DIR] [--label NAME] [--groups N] |\n"
                 "       rig-fit <calibration dir> <dir>... |\n"
                 "       board-check <calibration dir> <dir>... |\n"
                 "       board-poses <calibration> <dir>... [--pad N] |\n"
                 "       calib-solve <captures dir> [--reference <calibration>] [--save <file>] [--no-distortion | --distortion-from <calibration>] |\n"
                 "       fixture-pack --out DIR [--mustang <Project1.ir_E10_prj>] [--stl mesh.stl] [--board <dir>] |\n"
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
    auto dev = device::EinstarDevice::connect(sim::make_sim_scanner().transport, opts);
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

std::unique_ptr<device::EinstarDevice> open_scanner(int heartbeat_ms, std::function<void(std::string_view)> transcript = {}) {
    auto devices = usb::enumerate_devices();
    if (!devices || devices->empty()) {
        std::println(stderr, "no Shining3D devices (vid 3267) found");
        return nullptr;
    }
    const usb::UsbDeviceInfo info = devices->front();
    auto transport = usb::open_libusb(info);
    if (!transport) {
        std::println(stderr, "open failed: {}", transport.error().message);
        return nullptr;
    }
    device::ConnectOptions opts;
    opts.heartbeat_ms = heartbeat_ms;
    opts.reopen = [info] { return usb::reopen_libusb(info); };
    if (transcript) {
        opts.verbose_transcript = true;
        opts.transcript_sink = std::move(transcript);
    }
    auto dev = device::EinstarDevice::connect(std::move(*transport), opts);
    if (!dev) {
        std::println(stderr, "connect failed: {}", dev.error().message);
        return nullptr;
    }
    return std::move(*dev);
}

int calib_dump_cmd(const char* dir) {
    auto dev = open_scanner(0);
    if (!dev) return 1;
    auto blob = dev->read_flash(0, calib::kFlashBlobSize);
    if (!blob) {
        std::println(stderr, "flash read failed: {}", blob.error().message);
        return 1;
    }
    auto files = calib::extract_ccf_files(*blob);
    if (!files) {
        std::println(stderr, "{}", files.error().message);
        return 1;
    }
    if (auto r = calib::write_ccf_directory(*files, dir); !r) {
        std::println(stderr, "{}", r.error().message);
        return 1;
    }
    std::println("wrote {}/LeftCCF.txt, RightCCF.txt, TexCCF.txt (calibration {}, serial {})", dir, files->calibration_time,
                 dev->info().serial);
    return calib_cmd(dir);
}

void write_pgm(const std::filesystem::path& path, const ImageU8& img) {
    std::ofstream f(path, std::ios::binary);
    f << "P5\n" << img.width() << " " << img.height() << "\n255\n";
    for (int y = 0; y < img.height(); ++y) f.write(reinterpret_cast<const char*>(img.view().row(y)), img.width());
}

struct ImageStats {
    double mean = 0;
    int p99 = 0;
    double saturated = 0, dark = 0;  // fractions (255, < 10)
};
ImageStats image_stats(const ImageU8& img) {
    std::array<std::uint64_t, 256> hist{};
    for (const auto v : img.pixels()) ++hist[v];
    ImageStats s;
    const double n = static_cast<double>(img.size());
    std::uint64_t acc = 0;
    bool have_p99 = false;
    for (int v = 0; v < 256; ++v) {
        s.mean += v * static_cast<double>(hist[static_cast<std::size_t>(v)]) / n;
        acc += hist[static_cast<std::size_t>(v)];
        if (!have_p99 && static_cast<double>(acc) >= 0.99 * n) s.p99 = v, have_p99 = true;
        if (v < 10) s.dark += static_cast<double>(hist[static_cast<std::size_t>(v)]) / n;
    }
    s.saturated = static_cast<double>(hist[255]) / n;
    return s;
}

int hw_test(std::span<char*> args) {
    const std::filesystem::path out = arg_str(args, "--out") ? arg_str(args, "--out") : "hw_test";
    const double seconds = arg_double(args, "--seconds", 6.0);
    const double texture_seconds = arg_double(args, "--texture-seconds", 3.0);
    const double button_seconds = arg_double(args, "--buttons", 0.0);
    std::filesystem::create_directories(out);
    int failures = 0;
    auto check = [&](bool ok, std::string_view what, const std::string& detail = {}) {
        std::println("  [{}] {}{}{}", ok ? "PASS" : "FAIL", what, detail.empty() ? "" : ": ", detail);
        if (!ok) ++failures;
    };

    std::println("== connect / identification ==");
    // Every request / reply goes to transcript.txt (timestamped).
    auto transcript_file = std::make_shared<std::ofstream>(out / "transcript.txt");
    auto transcript_mutex = std::make_shared<std::mutex>();
    auto transcript_clock = std::make_shared<Stopwatch>();
    auto dev = open_scanner(250, [transcript_file, transcript_mutex, transcript_clock](std::string_view line) {  // heartbeat on: state polling, buttons
        std::lock_guard lock(*transcript_mutex);
        *transcript_file << std::format("{:9.3f} {}\n", transcript_clock->elapsed_ms() / 1000.0, line);
    });
    if (!dev) return 1;
    print_device(*dev);
    const auto& info = dev->info();
    check(info.sensor_count == 3, "three sensors", std::format("{}", info.sensor_count));
    for (int i = 0; i < info.sensor_count; ++i) {
        const auto& sn = info.sensors[static_cast<std::size_t>(i)];
        check(sn.width == 1280 && sn.height == 1024 && sn.pixel_bits == 8, std::format("sensor {} geometry", i),
              std::format("{}x{} {}-bit", sn.width, sn.height, sn.pixel_bits));
    }

    std::println("== flash / calibration ==");
    auto blob = dev->read_flash(0, calib::kFlashBlobSize);
    auto blob2 = dev->read_flash(0, calib::kFlashBlobSize);
    check(blob && blob2 && *blob == *blob2, "flash blob reads are consistent");
    std::optional<calib::DeviceCalibration> cal;
    if (blob) {
        std::ofstream(out / "flash_blob.bin", std::ios::binary).write(reinterpret_cast<const char*>(blob->data()), static_cast<std::streamsize>(blob->size()));
        auto c = calib::decode_flash_blob(*blob);
        check(c.has_value(), "calibration decodes", c ? std::format("{}, baseline {:.3f} mm", c->calibration_time, c->rig().baseline_mm()) : c.error().message);
        if (c) cal = *c;
        auto files = calib::extract_ccf_files(*blob);
        if (files && calib::write_ccf_directory(*files, (out / "calibration").string())) {
            auto reloaded = calib::load_ccf_directory((out / "calibration").string());
            check(reloaded && c && reloaded->rig().baseline_mm() == c->rig().baseline_mm(), "CCF files round trip",
                  (out / "calibration").string());
        }
    }

    std::println("== registers (sensors 0 and 1 share one exposure register) ==");
    for (int sensor = 0; sensor < 3; ++sensor) {
        const auto e = dev->exposure(sensor);
        const auto g = dev->gain(sensor);
        if (!e || !g) {
            check(false, std::format("sensor {} exposure/gain read", sensor));
            continue;
        }
        const std::uint32_t e2 = *e == 4000 ? 4100 : 4000;
        // Multiples of 25 %: the firmware stores gain in 1/32 steps, so other values read back rounded
        // (110 -> 109, 120 -> 119).
        const std::uint16_t g2 = *g == 100 ? 125 : 100;
        const bool w1 = dev->set_exposure(sensor, e2) && dev->set_gain(sensor, g2);
        const auto e_rb = dev->exposure(sensor);
        const auto g_rb = dev->gain(sensor);
        const bool w2 = dev->set_exposure(sensor, *e) && dev->set_gain(sensor, *g);  // restore
        const auto e_back = dev->exposure(sensor);
        check(w1 && w2 && e_rb && *e_rb == e2 && g_rb && *g_rb == g2 && e_back && *e_back == *e,
              std::format("sensor {} exposure/gain write + read back", sensor),
              std::format("was {}/{}, wrote {}/{}, read {}/{}", *e, *g, e2, g2, e_rb ? *e_rb : 0u, g_rb ? *g_rb : 0));
    }
    if (auto t = dev->temperature_c()) std::println("  temperature (idle) {:.2f} C", *t);
    auto st = dev->read_state();
    check(st.has_value(), "device state / buttons read");

    std::println("== indicator LEDs (watch the scanner: zone 0, 1, 2, then 1) ==");
    for (int zone = 0; zone < 3; ++zone) {
        const bool ok = dev->set_indication(static_cast<device::DistanceIndication>(zone)).has_value();
        check(ok, std::format("indication zone {} on", zone));
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
    check(dev->set_indication(device::DistanceIndication::zone1).has_value(), "indication back to zone 1");

    // Scan-mode streaming with the app's capture settings.
    struct Capture {
        std::mutex m;
        std::vector<usb::FrameGroup> groups;         // first frames, kept for analysis
        std::vector<double> arrival_ms;
        std::vector<std::uint32_t> ids;
        std::vector<unsigned> masks;
        Stopwatch clock;
    };
    auto stream = [&](const std::string& label, int laser, int strobe, double secs, bool texture, std::size_t keep,
                      std::uint32_t exposure = 4400, std::uint16_t gain = 120, bool quiet = false) {
        auto cap = std::make_shared<Capture>();
        if (auto* f = transcript_file.get()) {
            std::lock_guard lock(*transcript_mutex);
            *f << std::format("---- {} (laser {}, strobe {}, exposure {}, gain {}) ----\n", label, laser, strobe, exposure, gain);
        }
        (void)dev->set_exposure(0, exposure);  // (shared by the IR pair)
        for (int sensor = 0; sensor < 2; ++sensor) (void)dev->set_gain(sensor, gain);
        (void)dev->set_laser_percent(laser);
        (void)dev->set_strobe(0, strobe);
        const auto mode = texture ? dev->configure_texture_mode(100000) : dev->configure_scan_mode(68000);
        if (!quiet) check(mode.has_value(), std::format("{}: trigger configured", label));
        auto r = dev->start_stream([cap, keep](usb::FrameGroup&& g) {
            std::lock_guard lock(cap->m);
            cap->arrival_ms.push_back(cap->clock.elapsed_ms());
            cap->ids.push_back(g.frame_id);
            cap->masks.push_back(g.mask());
            if (cap->groups.size() < keep) cap->groups.push_back(std::move(g));
        });
        if (!quiet || !r) check(r.has_value(), std::format("{}: stream started", label), r ? "" : r.error().message);
        std::vector<double> temps;
        for (double t = 0; t < secs; t += 1.0) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (auto c = dev->temperature_c()) temps.push_back(*c);
        }
        (void)dev->set_trigger(0, 0);
        (void)dev->set_laser_percent(0);
        (void)dev->set_strobe(0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        dev->stop_stream();
        const auto ss = dev->stream_stats();
        const auto gs = dev->group_stats();
        std::lock_guard lock(cap->m);
        const std::size_t n = cap->ids.size();
        const double span = n > 1 ? cap->arrival_ms.back() - cap->arrival_ms.front() : 0.0;
        const double hz = n > 1 ? 1000.0 * static_cast<double>(n - 1) / span : 0.0;
        std::uint64_t gaps = 0;
        for (std::size_t i = 1; i < n; ++i)
            if (cap->ids[i] != cap->ids[i - 1] + 1) gaps += cap->ids[i] > cap->ids[i - 1] ? cap->ids[i] - cap->ids[i - 1] - 1 : 1;
        std::vector<double> dt;
        for (std::size_t i = 1; i < n; ++i) dt.push_back(cap->arrival_ms[i] - cap->arrival_ms[i - 1]);
        std::ranges::sort(dt);
        if (quiet) return std::move(cap->groups);
        // (The assemblers and their statistics are per stream.)
        std::println("  {} groups in {:.1f} s = {:.2f} Hz; arrival interval median {:.1f} ms, max {:.1f} ms; frame-id gaps {}", n,
                     secs, hz, dt.empty() ? 0.0 : dt[dt.size() / 2], dt.empty() ? 0.0 : dt.back(), gaps);
        std::println("  packets {} (longest {} bytes; the firmware's DMA buffer allows {}), frames {}, bad packets {}, resyncs {}, "
                     "overflows {}, bad camera {}; groups {}, incomplete dropped {}, IR-only {}",
                     ss.packets, ss.max_packet, usb::kStreamDeviceBufferSize, ss.frames, ss.bad_packets, ss.resyncs, ss.overflows,
                     ss.bad_camera, gs.groups, gs.incomplete_dropped, gs.ir_only_emitted);
        const auto ts = dev->transport_stats();
        if (ts.stream_stalls > 0 || dev->reconnects() > 0)
            std::println("  image-endpoint stalls cleared {}, reconnections {}", ts.stream_stalls, dev->reconnects());
        if (!temps.empty()) std::println("  temperature while streaming {:.2f} .. {:.2f} C", std::ranges::min(temps), std::ranges::max(temps));
        std::map<unsigned, int> mask_counts;
        for (const auto m : cap->masks) ++mask_counts[m];
        for (const auto& [m, c] : mask_counts) std::println("  sensor mask {:03b}: {} groups", m, c);
        const double expected_hz = texture ? 10.0 : 1e6 / 68000.0;
        check(n > 0 && std::abs(hz - expected_hz) < 0.1 * expected_hz, std::format("{}: frame rate", label),
              std::format("{:.2f} Hz (trigger period implies {:.2f})", hz, expected_hz));
        check(ss.resyncs == 0 && ss.bad_packets == 0 && ss.overflows == 0 && gaps == 0, std::format("{}: clean stream", label));
        std::vector<usb::FrameGroup> groups = std::move(cap->groups);
        return groups;
    };
    auto report_images = [&](const std::vector<usb::FrameGroup>& groups, const std::string& tag) {
        if (groups.empty()) return;
        const auto& g = groups[groups.size() / 2];
        for (int s = 0; s < 3; ++s) {
            if (!g.sensors[static_cast<std::size_t>(s)]) continue;
            const auto& img = g.sensors[static_cast<std::size_t>(s)]->pixels;
            const auto is = image_stats(img);
            std::println("  sensor {} (frame {}): mean {:.1f}, p99 {}, saturated {:.2f}%, dark {:.1f}%", s, g.frame_id, is.mean, is.p99,
                         100 * is.saturated, 100 * is.dark);
            write_pgm(out / std::format("{}_sensor{}.pgm", tag, s), img);
        }
    };

    // What the light sources and the exposure register do, measured on the images (scene held still).
    auto mean_of = [](const std::vector<usb::FrameGroup>& groups, int sensor) {
        if (groups.empty() || !groups.back().sensors[static_cast<std::size_t>(sensor)]) return -1.0;
        return image_stats(groups.back().sensors[static_cast<std::size_t>(sensor)]->pixels).mean;
    };
    std::println("== light sources (mean IR level, sensors 0 / 1; exposure 4400, gain 120) ==");
    for (const auto& [name, laser, strobe] : {std::tuple{"all off", 0, 0}, std::tuple{"projector 100", 100, 0},
                                              std::tuple{"strobe 6000", 0, 6000}, std::tuple{"both", 100, 6000}}) {
        const auto g = stream(name, laser, strobe, 1.0, false, 8, 4400, 120, true);
        std::println("  {:14} {:6.1f} / {:6.1f}", name, mean_of(g, 0), mean_of(g, 1));
        if (!g.empty()) write_pgm(out / std::format("light_{}_sensor0.pgm", laser * 100000 + strobe), g.back().sensors[0]->pixels);
    }
    // 10/62 DISTANCE selects the FPGA laser mode (0/1/2 -> modes 4/1/2, docs/firmware.md 5): what the
    // "LD" level (10/68) and the strobe do in each. Detail = mean |Laplacian| (speckle shows up there).
    auto detail_of = [](const std::vector<usb::FrameGroup>& groups, int sensor) {
        if (groups.empty() || !groups.back().sensors[static_cast<std::size_t>(sensor)]) return -1.0;
        const auto& img = groups.back().sensors[static_cast<std::size_t>(sensor)]->pixels;
        double acc = 0;
        std::size_t n = 0;
        for (int y = 1; y + 1 < img.height(); y += 2)
            for (int x = 1; x + 1 < img.width(); x += 2) {
                const int c = img(x, y);
                acc += std::abs(4 * c - img(x - 1, y) - img(x + 1, y) - img(x, y - 1) - img(x, y + 1));
                ++n;
            }
        return acc / static_cast<double>(n);
    };
    std::println("== laser mode x level (sensor 0: mean / detail) ==");
    std::println("  {:10} {:>14} {:>14} {:>14} {:>14}", "DISTANCE", "off", "LD 100", "strobe 6000", "both");
    for (int zone = 0; zone < 3; ++zone) {
        (void)dev->set_indication(static_cast<device::DistanceIndication>(zone));
        std::string row = std::format("  {:<10}", zone);
        for (const auto& [laser, strobe] : {std::pair{0, 0}, std::pair{100, 0}, std::pair{0, 6000}, std::pair{100, 6000}}) {
            const auto g = stream("laser mode", laser, strobe, 1.0, false, 8, 4400, 120, true);
            row += std::format(" {:6.1f} / {:5.1f}", mean_of(g, 0), detail_of(g, 0));
            if (!g.empty()) write_pgm(out / std::format("mode{}_ld{}_strobe{}_sensor0.pgm", zone, laser, strobe), g.back().sensors[0]->pixels);
        }
        std::println("{}", row);
    }
    (void)dev->set_indication(device::DistanceIndication::zone2);  // the firmware's start-up laser mode

    std::println("== exposure / gain (lights off: ambient only; mean IR level, sensors 0 / 1) ==");
    for (const auto& [e, gn] : {std::pair{250u, 120}, std::pair{1000u, 120}, std::pair{4000u, 120}, std::pair{1000u, 30}, std::pair{1000u, 480}}) {
        const auto g = stream("exposure", 0, 0, 1.0, false, 8, e, static_cast<std::uint16_t>(gn), true);
        std::println("  exposure {:5} gain {:3}: {:6.1f} / {:6.1f}", e, gn, mean_of(g, 0), mean_of(g, 1));
    }

    std::println("== scan stream (EXStar-level laser 60, strobe 6000) ==");
    auto scan60 = stream("scan L60", 60, 6000, 2.0, false, 4);
    report_images(scan60, "scan_laser60");
    std::println("== scan stream (app default laser 100, strobe 6000) ==");
    auto scan = stream("scan L100", 100, 6000, seconds, false, 40);
    report_images(scan, "scan_laser100");

    if (cal && !scan.empty()) {
        std::println("== depth and markers on the real frames ==");
        pipeline::StereoFrontend fe(cal->rig());
        fe.detect_sensor_order(scan.front());
        std::vector<double> valid, stereo_ms;
        std::vector<int> left_markers, stereo_markers;
        for (const auto& g : scan) {
            auto d = fe.process(g);
            if (!d) continue;
            d->frame.ensure_cpu();
            const auto& pts = d->frame.points;
            if (&g == &scan.front()) {
                // Depth of the first frame (150..700 mm -> bright..dark, black = none) and its z range.
                ImageU8 depth_img(pts.width(), pts.height());
                float zmin = 1e9f, zmax = 0;
                for (int y = 0; y < pts.height(); ++y)
                    for (int x = 0; x < pts.width(); ++x) {
                        const float z = pts(x, y).z();
                        depth_img(x, y) = z > 0 ? static_cast<std::uint8_t>(std::clamp(255.0f - (z - 150.0f) * 200.0f / 550.0f, 30.0f, 255.0f)) : 0;
                        if (z > 0) zmin = std::min(zmin, z), zmax = std::max(zmax, z);
                    }
                write_pgm(out / "depth_frame0.pgm", depth_img);
                std::println("  frame 0: depth {:.0f}..{:.0f} mm; {} stereo markers, {} left detections unmatched", zmin, zmax,
                             d->markers.size(), d->unmatched_left.size());
            }
            valid.push_back(static_cast<double>(std::ranges::count_if(pts.pixels(), [](const Vec3f& p) { return p.z() > 0; })) /
                            static_cast<double>(pts.size()));
            stereo_ms.push_back(d->stereo_ms);
            stereo_markers.push_back(static_cast<int>(d->markers.size()));
            left_markers.push_back(static_cast<int>(d->markers.size() + d->unmatched_left.size()));
        }
        auto median = [](auto v) {
            using T = typename decltype(v)::value_type;
            std::ranges::sort(v);
            return v.empty() ? T{} : v[v.size() / 2];
        };
        std::println("  {} frames: valid depth median {:.1f}% (min {:.1f}%), stereo {:.1f} ms; markers median {} in the left image, {} stereo",
                     valid.size(), 100 * median(valid), 100 * (valid.empty() ? 0.0 : std::ranges::min(valid)), median(stereo_ms),
                     median(left_markers), median(stereo_markers));
        check(!valid.empty() && median(valid) > 0.05, "depth from real IR");

        {  // The rectified pair of the first frame (half resolution), for offline checks.
            pipeline::StereoFrontendParams rp;
            rp.cpu_previews = true;
            pipeline::StereoFrontend rfe(cal->rig(), rp);
            if (auto d = rfe.process(scan.front().sensors[0]->pixels, scan.front().sensors[1]->pixels); !d.rectified_left.empty()) {
                write_pgm(out / "rectified_left.pgm", d.rectified_left);
                write_pgm(out / "rectified_right.pgm", d.rectified_right);
            }
        }
        // Epipolar check on the markers: after rectification, a marker must sit on the same row in both
        // images (SGM and the marker matcher search along rows).
        {
            const auto& g = scan.front();
            const auto l = markers::detect_markers(g.sensors[0]->pixels.view());
            const auto r = markers::detect_markers(g.sensors[1]->pixels.view());
            const auto& ms = fe.marker_stereo();
            std::vector<double> ly, ry;
            for (const auto& e : l) ly.push_back(ms.rectify_left(e.center).y());
            for (const auto& e : r) ry.push_back(ms.rectify_right(e.center).y());
            std::vector<double> dy;
            for (const double y : ly) {
                double best = 1e9;
                for (const double y2 : ry) best = std::abs(y2 - y) < std::abs(best) ? y2 - y : best;
                if (std::abs(best) < 1e8) dy.push_back(best);
            }
            std::ranges::sort(dy);
            std::print("  markers: {} left, {} right; rectified row offset to the nearest right marker (px):", l.size(), r.size());
            for (const double d : dy) std::print(" {:.1f}", d);
            std::println("");
        }

        // What exposure and gain do with the scanner's own light (projector + strobe on): brightness,
        // valid depth and stereo markers per setting.
        std::println("== exposure / gain with the projector and strobe on (mean IR / valid depth / stereo markers) ==");
        for (const auto& [e, gn] : {std::pair{1100u, 120}, std::pair{2200u, 120}, std::pair{4400u, 120}, std::pair{8800u, 120},
                                    std::pair{4400u, 240}, std::pair{4400u, 480}, std::pair{8800u, 240}}) {
            const auto groups = stream("brightness", 100, 6000, 1.0, false, 6, e, static_cast<std::uint16_t>(gn), true);
            if (groups.empty()) continue;
            std::vector<double> v;
            std::vector<int> mk;
            for (std::size_t i = std::min<std::size_t>(2, groups.size() - 1); i < groups.size(); ++i) {  // (settled frames)
                auto d = fe.process(groups[i]);
                if (!d) continue;
                d->frame.ensure_cpu();
                v.push_back(static_cast<double>(std::ranges::count_if(d->frame.points.pixels(), [](const Vec3f& p) { return p.z() > 0; })) /
                            static_cast<double>(d->frame.points.size()));
                mk.push_back(static_cast<int>(d->markers.size()));
            }
            std::println("  exposure {:5} gain {:3}: {:6.1f} / {:5.1f}% / {}", e, gn, mean_of(groups, 0), 100 * median(v), median(mk));
            write_pgm(out / std::format("bright_e{}_g{}_sensor0.pgm", e, gn), groups.back().sensors[0]->pixels);
        }

        // Image orientation: which flip of each IR image matches the flash calibration? (On the first
        // hardware run sensor 1 looked vertically mirrored relative to sensors 0 and 2.)
        std::println("== IR image orientation vs the calibration (valid depth / stereo markers, median of 10 frames) ==");
        auto flipped = [](const ImageU8& in, bool vertical, bool horizontal) {
            ImageU8 o(in.width(), in.height());
            for (int y = 0; y < in.height(); ++y)
                for (int x = 0; x < in.width(); ++x) o(x, y) = in(horizontal ? in.width() - 1 - x : x, vertical ? in.height() - 1 - y : y);
            return o;
        };
        const std::pair<const char*, std::pair<bool, bool>> variants[] = {
            {"as delivered", {false, false}}, {"vertical flip", {true, false}}, {"horizontal flip", {false, true}}, {"rotate 180", {true, true}}};
        for (int which = 0; which < 2; ++which)
            for (const auto& [name, vh] : variants) {
                if (which == 1 && !vh.first && !vh.second) continue;  // (same as sensor 0's "as delivered")
                std::vector<double> v;
                std::vector<int> mk;
                for (std::size_t i = 0; i < std::min<std::size_t>(scan.size(), 10); ++i) {
                    const auto& g = scan[i];
                    ImageU8 l = g.sensors[0]->pixels, r = g.sensors[1]->pixels;
                    (which == 0 ? r : l) = flipped(which == 0 ? r : l, vh.first, vh.second);
                    auto d = fe.process(l, r);
                    d.frame.ensure_cpu();
                    v.push_back(static_cast<double>(std::ranges::count_if(d.frame.points.pixels(), [](const Vec3f& p) { return p.z() > 0; })) /
                                static_cast<double>(d.frame.points.size()));
                    mk.push_back(static_cast<int>(d.markers.size()));
                }
                std::println("  sensor {} {:16} {:5.1f}% / {}", which == 0 ? 1 : 0, name, 100 * median(v), median(mk));
            }
    }

    if (texture_seconds > 0) {
        std::println("== texture stream (1 IR + RGB) ==");
        auto tex = stream("texture", 60, 6000, texture_seconds, true, 4);
        report_images(tex, "texture");
    }

    if (button_seconds > 0) {
        std::println("== buttons: press each scanner button (short and long) within {:.0f} s ==", button_seconds);
        std::mutex bm;
        std::vector<std::string> events;
        Stopwatch bclock;
        dev->set_button_sink([&](int button, device::ButtonAction action) {
            std::lock_guard lock(bm);
            events.push_back(std::format("{:.1f} s: button {} action {}", bclock.elapsed_ms() / 1000.0, button, static_cast<int>(action)));
            std::println("  {}", events.back());
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(button_seconds * 1000)));
        dev->set_button_sink({});
        check(!events.empty(), "button events received", std::format("{}", events.size()));
    }

    dev->disconnect();  // triggers, laser and strobe off
    std::println("== {} ({} failures); images and calibration in {} ==", failures ? "FAILED" : "all passed", failures, out.string());
    return failures ? 1 : 0;
}

// Indicator LEDs and buttons, with someone at the scanner following a fixed timeline: a countdown, the
// three DISTANCE values idle and then while scanning, then a window for button presses. Every change of
// the device-state reply (00/07) is printed with its time.
int hw_ui(std::span<char*> args) {
    const double countdown = arg_double(args, "--countdown", 10.0);
    const double led_seconds = arg_double(args, "--led-seconds", 5.0);
    const double button_seconds = arg_double(args, "--button-seconds", 40.0);
    auto dev = open_scanner(0);  // no heartbeat: this polls the state itself
    if (!dev) return 1;
    Stopwatch clock;
    auto now = [&] { return clock.elapsed_ms() / 1000.0; };
    auto wait_until = [&](double t) {
        while (now() < t) std::this_thread::sleep_for(std::chrono::milliseconds(20));
    };
    auto hex = [](const std::array<std::uint8_t, 14>& b) {
        std::string s;
        for (const auto v : b) s += std::format("{:02X} ", v);
        return s;
    };
    std::println("{:6.1f} s  countdown {:.0f} s", now(), countdown);
    wait_until(countdown);
    double t = countdown;
    std::println("{:6.1f} s  == indicator, idle ==", now());
    for (int zone = 0; zone < 3; ++zone) {
        const bool ok = dev->set_indication(static_cast<device::DistanceIndication>(zone)).has_value();
        std::println("{:6.1f} s  DISTANCE {} {}", now(), zone, ok ? "" : "(failed)");
        wait_until(t += led_seconds);
    }
    std::println("{:6.1f} s  == indicator, scanning (projector and strobe on) ==", now());
    (void)dev->set_exposure(0, 4400);
    for (int sensor = 0; sensor < 2; ++sensor) (void)dev->set_gain(sensor, 120);
    (void)dev->set_laser_percent(100);
    (void)dev->set_strobe(0, 6000);
    (void)dev->configure_scan_mode(68000);
    std::atomic<int> groups{0};
    (void)dev->start_stream([&](usb::FrameGroup&&) { ++groups; });
    for (int zone = 0; zone < 3; ++zone) {
        const bool ok = dev->set_indication(static_cast<device::DistanceIndication>(zone)).has_value();
        std::println("{:6.1f} s  DISTANCE {} {}", now(), zone, ok ? "" : "(failed)");
        wait_until(t += led_seconds);
    }
    (void)dev->set_trigger(0, 0);
    (void)dev->set_laser_percent(0);
    (void)dev->set_strobe(0, 0);
    dev->stop_stream();
    (void)dev->set_indication(device::DistanceIndication::zone2);  // the firmware's start-up laser mode
    std::println("{:6.1f} s  scanning stopped ({} frames)", now(), groups.load());

    std::println("{:6.1f} s  == buttons: state bytes 9..22 on every change (polled every 50 ms) ==", now());
    std::optional<std::array<std::uint8_t, 14>> last;
    int polls = 0, changes = 0;
    const double end = t + button_seconds;
    while (now() < end) {
        if (auto st = dev->read_state()) {
            ++polls;
            if (!last || st->raw != *last) {
                ++changes;
                std::print("{:6.1f} s  {}", now(), hex(st->raw));
                for (int b = 0; b < 3; ++b)
                    if (st->buttons[static_cast<std::size_t>(b)] != device::ButtonAction::none)
                        std::print(" button {} code {}", b, static_cast<int>(st->buttons[static_cast<std::size_t>(b)]));
                std::println("");
                last = st->raw;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::println("{:6.1f} s  done: {} state reads, {} changes", now(), polls, changes);
    return 0;
}

// Each light source on its own while streaming, on a fixed timeline for someone watching the scanner:
// IR and RGB levels, marker brightness (the brightest 0.1% of IR pixels) and images per step.
int hw_lights(std::span<char*> args) {
    namespace fs = std::filesystem;
    const fs::path out = arg_str(args, "--out") ? arg_str(args, "--out") : "hw_lights";
    const double countdown = arg_double(args, "--countdown", 10.0);
    const double step_seconds = arg_double(args, "--step-seconds", 5.0);
    fs::create_directories(out);
    auto dev = open_scanner(250);
    if (!dev) return 1;
    struct Step { const char* name; bool texture; int laser; int strobe0; int strobe1; };
    const Step steps[] = {
        {"all off", true, 0, 0, 0},           {"strobe route 0 = 6000", true, 0, 6000, 0}, {"strobe route 0 = 9000", true, 0, 9000, 0},
        {"strobe route 1 = 6000", true, 0, 0, 6000}, {"strobe route 1 = 9000", true, 0, 0, 9000}, {"LD 100 (projector)", true, 100, 0, 0},
        {"scan mode: LD 100 + route 0 6000", false, 100, 6000, 0}, {"scan mode: route 1 9000 only", false, 0, 0, 9000},
    };
    Stopwatch clock;
    auto now = [&] { return clock.elapsed_ms() / 1000.0; };
    std::println("{:6.1f} s  countdown {:.0f} s", now(), countdown);
    while (now() < countdown) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    (void)dev->set_exposure(0, 4400);
    for (int sensor = 0; sensor < 2; ++sensor) (void)dev->set_gain(sensor, 120);
    std::println("{:36} {:>8} {:>8} {:>9} {:>8}", "step (from)", "IR mean", "IR p99.9", "IR >200", "RGB mean");
    int index = 0;
    for (const auto& st : steps) {
        const double start = now();
        std::mutex m;
        std::optional<usb::FrameGroup> last;
        int groups = 0;
        (void)dev->set_laser_percent(st.laser);
        (void)dev->set_strobe(0, st.strobe0);
        (void)dev->set_strobe(1, st.strobe1);
        (void)(st.texture ? dev->configure_texture_mode(100000) : dev->configure_scan_mode(68000));
        (void)dev->start_stream([&](usb::FrameGroup&& g) {
            std::lock_guard lock(m);
            ++groups;
            last = std::move(g);
        });
        while (now() < start + step_seconds) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        (void)dev->set_trigger(0, 0);
        (void)dev->set_laser_percent(0);
        (void)dev->set_strobe(0, 0);
        (void)dev->set_strobe(1, 0);
        dev->stop_stream();
        std::lock_guard lock(m);
        double ir_mean = -1, rgb_mean = -1;
        int p999 = -1;
        long bright = -1;
        if (last && last->sensors[0]) {
            const auto& ir = last->sensors[0]->pixels;
            std::array<long, 256> hist{};
            for (const auto v : ir.pixels()) ++hist[v];
            ir_mean = image_stats(ir).mean;
            long acc = 0;
            for (int v = 0; v < 256 && p999 < 0; ++v)
                if ((acc += hist[static_cast<std::size_t>(v)]) >= static_cast<long>(0.999 * static_cast<double>(ir.size()))) p999 = v;
            bright = 0;
            for (int v = 201; v < 256; ++v) bright += hist[static_cast<std::size_t>(v)];
            write_pgm(out / std::format("step{}_ir0.pgm", index), ir);
        }
        if (last && last->sensors[2]) {
            rgb_mean = image_stats(last->sensors[2]->pixels).mean;
            write_pgm(out / std::format("step{}_rgb.pgm", index), last->sensors[2]->pixels);
        }
        std::println("{:28} {:5.1f}s {:8.1f} {:8} {:9} {:8.1f}  ({} groups)", st.name, start, ir_mean, p999, bright, rgb_mean, groups);
        ++index;
    }
    std::println("{:6.1f} s  done", now());
    return 0;
}

std::optional<ImageU8> read_pgm(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    std::string magic;
    int w = 0, h = 0, maxval = 0;
    f >> magic >> w >> h >> maxval;
    if (magic != "P5" || w <= 0 || h <= 0 || maxval != 255) return std::nullopt;
    f.get();
    ImageU8 img(w, h);
    f.read(reinterpret_cast<char*>(img.data()), static_cast<std::streamsize>(img.size()));
    if (!f) return std::nullopt;
    return img;
}

// Marker detection on a saved 8-bit IR frame (PGM): candidate blobs and why each was rejected.
int markers_debug(std::span<char*> args) {
    if (args.empty()) return usage();
    auto img = read_pgm(args[0]);
    if (!img) {
        std::println(stderr, "cannot read {} (8-bit binary PGM)", args[0]);
        return 1;
    }
    markers::DetectParams p;
    p.threshold = static_cast<int>(arg_int(args, "--threshold", p.threshold));
    p.min_diameter_px = arg_double(args, "--min-diameter", p.min_diameter_px);
    p.ring_scale = arg_double(args, "--ring-scale", p.ring_scale);
    p.ring_max_contrast = arg_double(args, "--ring-contrast", p.ring_max_contrast);
    p.max_ring_bright_fraction = arg_double(args, "--ring-bright", p.max_ring_bright_fraction);
    p.saturated_ring_max_contrast = arg_double(args, "--sat-ring-contrast", p.saturated_ring_max_contrast);
    p.saturated_max_ring_bright_fraction = arg_double(args, "--sat-ring-bright", p.saturated_max_ring_bright_fraction);
    const auto blobs = markers::find_blobs_cpu(img->view(), p.threshold);
    markers::FitStats stats;
    const auto found = markers::fit_blobs(img->view(), blobs, p, &stats);
    std::println("{} blobs above {}, {} markers", blobs.size(), p.threshold, found.size());
    for (std::size_t r = 0; r < stats.count.size(); ++r)
        if (stats.count[r] > 0) std::println("  {:14} {}", markers::reject_name(static_cast<markers::Reject>(r)), stats.count[r]);
    for (const auto& e : found)
        std::println("  marker at ({:.1f}, {:.1f}) a {:.2f} b {:.2f} residual {:.3f} peak {}", e.center.x(), e.center.y(), e.a, e.b,
                     e.residual, e.peak);
    // The larger candidates (markers are the biggest bright blobs), one at a time, with their reason.
    std::vector<markers::Blob> big;
    for (const auto& b : blobs)
        if (b.pixels >= 30) big.push_back(b);
    std::ranges::sort(big, [](const markers::Blob& a, const markers::Blob& b) { return a.pixels > b.pixels; });
    for (std::size_t i = 0; i < std::min<std::size_t>(big.size(), 20); ++i) {
        markers::FitStats one;
        (void)markers::fit_blobs(img->view(), {big[i]}, p, &one);
        const auto r = static_cast<markers::Reject>(std::ranges::max_element(one.count) - one.count.begin());
        std::println("  blob ({:4},{:4})-({:4},{:4}) {:5} px peak {:3}: {}", big[i].x0, big[i].y0, big[i].x1, big[i].y1, big[i].pixels,
                     big[i].peak, markers::reject_name(r));
    }
    return 0;
}

// A calibration argument: a CCF directory, a flash blob (.bin: its quick CCF section) or
// "factory:<flash blob>" (its factory section).
Result<calib::DeviceCalibration> load_calibration(std::string_view arg) {
    if (arg.starts_with("rig:")) {  // written by board-check --ba --save-rig
        std::ifstream f(std::string(arg.substr(4)));
        calib::DeviceCalibration cal;
        for (auto* c : {&cal.left, &cal.right}) {
            auto& m = c->model;
            m.width = 1280, m.height = 1024;
            f >> m.fx >> m.fy >> m.cx >> m.cy >> m.skew;
            for (auto& d : m.dist) f >> d;
        }
        for (int i = 0; i < 9; ++i) f >> cal.right.R_cam_world(i / 3, i % 3);
        for (int i = 0; i < 3; ++i) f >> cal.right.t_cam_world(i);
        cal.texture = cal.left;
        if (!f) return make_error(Errc::invalid_argument, std::format("cannot read rig file {}", arg.substr(4)));
        return cal;
    }
    const bool factory = arg.starts_with("factory:");
    const std::string path(factory ? arg.substr(8) : arg);
    if (std::filesystem::is_regular_file(path) && !path.ends_with(".bin")) {  // einstar-calibrate's file
        auto file = calibrate::read_calibration_file(path);
        if (!file) return std::unexpected(file.error());
        calib::DeviceCalibration cal;
        cal.left.model = file->rig.left;
        cal.right.model = file->rig.right;
        cal.texture.model = file->rig.texture;
        cal.right.R_cam_world = file->rig.T_right_left.linear();
        cal.right.t_cam_world = file->rig.T_right_left.translation();
        cal.texture.R_cam_world = file->rig.T_texture_left.linear();
        cal.texture.t_cam_world = file->rig.T_texture_left.translation();
        cal.calibration_time = file->created;
        return cal;
    }
    if (!path.ends_with(".bin")) return calib::load_ccf_directory(path);
    std::ifstream f(path, std::ios::binary);
    std::vector<std::uint8_t> blob((std::istreambuf_iterator<char>(f)), {});
    return factory ? calib::decode_factory_section(blob) : calib::decode_flash_blob(blob);
}

// Stereo on a saved raw IR pair (PGM) with a calibration directory: valid depth and the rectified row
// offsets of markers matched by nearest row, as delivered and with both images turned 180 degrees.
int stereo_debug(std::span<char*> args) {
    if (args.size() < 3) return usage();
    auto l = read_pgm(args[0]), r = read_pgm(args[1]);
    auto cal = load_calibration(args[2]);
    if (!l || !r || !cal) {
        std::println(stderr, "need <left.pgm> <right.pgm> <calibration dir>");
        return 1;
    }
    pipeline::StereoFrontend fe(cal->rig());
    auto rotated = [](ImageU8 img) {
        std::ranges::reverse(img.pixels());
        return img;
    };
    if (has_flag(args, "--orientations")) {
        // Every flip / rotation of each image and both assignments of the cameras, scored by the row
        // error of the markers after rectification (the calibration's own convention fits to ~0.05 px).
        auto transformed = [](const ImageU8& in, int t) {  // 0 identity, 1 horizontal flip, 2 vertical flip, 3 rotate 180
            ImageU8 o(in.width(), in.height());
            const bool h = t == 1 || t == 3, v = t == 2 || t == 3;
            for (int y = 0; y < in.height(); ++y)
                for (int x = 0; x < in.width(); ++x) o(x, y) = in(h ? in.width() - 1 - x : x, v ? in.height() - 1 - y : y);
            return o;
        };
        const char* names[] = {"id", "hflip", "vflip", "rot180"};
        const auto& ms = fe.marker_stereo();
        struct Row { double rms; int matched; std::string what; };
        std::vector<Row> rows;
        for (int swap = 0; swap < 2; ++swap)
            for (int tl = 0; tl < 4; ++tl)
                for (int tr = 0; tr < 4; ++tr) {
                    const ImageU8 L = transformed(swap ? *r : *l, tl), R = transformed(swap ? *l : *r, tr);
                    const auto el = markers::detect_markers(L.view()), er = markers::detect_markers(R.view());
                    double ss = 0;
                    int n = 0;
                    for (const auto& a : el) {
                        const Vec2 ra = ms.rectify_left(a.center);
                        double best = 1e9;
                        for (const auto& b : er) {
                            const Vec2 rb = ms.rectify_right(b.center);
                            if (rb.x() < ra.x() && std::abs(rb.y() - ra.y()) < std::abs(best)) best = rb.y() - ra.y();
                        }
                        if (std::abs(best) < 30) ss += best * best, ++n;
                    }
                    rows.push_back({n ? std::sqrt(ss / n) : 1e9, n,
                                    std::format("left = {} {:6}, right = {} {:6}", swap ? "sensor 1" : "sensor 0", names[tl], swap ? "sensor 0" : "sensor 1",
                                                names[tr])});
                }
        std::ranges::sort(rows, [](const Row& a, const Row& b) { return a.matched * 1.0 / (1 + a.rms) > b.matched * 1.0 / (1 + b.rms); });
        for (const auto& row : rows) std::println("  {}: {} matched, row rms {:.3f} px", row.what, row.matched, row.rms);
        return 0;
    }
    for (const bool rot : {false, true}) {
        const ImageU8 L = rot ? rotated(*l) : *l, R = rot ? rotated(*r) : *r;
        auto d = fe.process(L, R);
        d.frame.ensure_cpu();
        const double valid = static_cast<double>(std::ranges::count_if(d.frame.points.pixels(), [](const Vec3f& p) { return p.z() > 0; })) /
                             static_cast<double>(d.frame.points.size());
        const auto el = markers::detect_markers(L.view()), er = markers::detect_markers(R.view());
        const auto& ms = fe.marker_stereo();
        std::vector<double> dy;
        for (const auto& a : el) {
            const Vec2 ra = ms.rectify_left(a.center);
            double best = 1e9;
            for (const auto& b : er) {
                const Vec2 rb = ms.rectify_right(b.center);
                if (rb.x() < ra.x() && std::abs(rb.y() - ra.y()) < std::abs(best)) best = rb.y() - ra.y();  // (right is left of left)
            }
            if (std::abs(best) < 20) {
                dy.push_back(best);
                if (!rot) std::println("  left marker raw ({:6.1f}, {:6.1f}) rectified ({:6.1f}, {:6.1f}): row offset {:+.2f}", a.center.x(), a.center.y(),
                                       ra.x(), ra.y(), best);
            }
        }
        if (!rot && has_flag(args, "--fit-shift")) {
            // Which shift of the raw right image makes the markers' rectified rows agree?
            std::vector<std::pair<Vec2, Vec2>> pairs;  // (left raw, right raw), matched as above
            for (const auto& a : el) {
                const Vec2 ra = ms.rectify_left(a.center);
                const markers::Ellipse* best = nullptr;
                double bd = 20;
                for (const auto& b : er) {
                    const Vec2 rb = ms.rectify_right(b.center);
                    if (rb.x() < ra.x() && std::abs(rb.y() - ra.y()) < bd) bd = std::abs(rb.y() - ra.y()), best = &b;
                }
                if (best) pairs.emplace_back(a.center, best->center);
            }
            double best_rms = 1e9, bx = 0, by = 0;
            for (double sx = -20; sx <= 20; sx += 0.25)
                for (double sy = -20; sy <= 20; sy += 0.25) {
                    double ss = 0;
                    for (const auto& [a, b] : pairs) {
                        const double e = ms.rectify_right(b + Vec2(sx, sy)).y() - ms.rectify_left(a).y();
                        ss += e * e;
                    }
                    const double rms = std::sqrt(ss / static_cast<double>(std::max<std::size_t>(pairs.size(), 1)));
                    if (rms < best_rms) best_rms = rms, bx = sx, by = sy;
                }
            std::println("  best shift of the raw right image: ({:+.2f}, {:+.2f}) px -> row rms {:.3f} px over {} markers", bx, by, best_rms, pairs.size());
        }
        std::ranges::sort(dy);
        std::print("{:18} valid depth {:5.1f}%, stereo markers {}, markers {} / {}; row offsets (px):", rot ? "both rotated 180" : "as delivered",
                   100 * valid, d.markers.size(), el.size(), er.size());
        for (const double v : dy) std::print(" {:.2f}", v);
        std::println("");
    }
    return 0;
}

// Raw captures for offline work: per pose (--label), a standard set of lighting conditions, each as
// PGM images (sensors 0 and 1 upright as the device delivers them, sensor 2 raw Bayer in texture mode)
// plus frames.csv (group id, per-sensor sub-field and timestamp) and the settings; the flash blob and
// calibration once per directory.
int hw_capture(std::span<char*> args) {
    namespace fs = std::filesystem;
    const fs::path root = arg_str(args, "--out") ? arg_str(args, "--out") : "fixtures-data/scanner";
    const std::string label = arg_str(args, "--label") ? arg_str(args, "--label") : "pose";
    const int scan_groups = static_cast<int>(arg_int(args, "--groups", 20));
    auto dev = open_scanner(250);
    if (!dev) return 1;
    fs::create_directories(root / label);
    if (!fs::exists(root / "flash_blob.bin"))
        if (auto blob = dev->read_flash(0, calib::kFlashBlobSize)) {
            std::ofstream(root / "flash_blob.bin", std::ios::binary).write(reinterpret_cast<const char*>(blob->data()), static_cast<std::streamsize>(blob->size()));
            if (auto files = calib::extract_ccf_files(*blob)) (void)calib::write_ccf_directory(*files, (root / "calibration").string());
            std::ofstream info(root / "device.txt");
            const auto& di = dev->info();
            info << "serial " << di.serial << "\nfirmware " << di.firmware << "\nnote sensor 1 frames are stored upright: the scanner sends them "
                 << "rotated 180 degrees (reverse the pixel order for the wire orientation)\n";
        }
    struct Condition { const char* name; bool texture; int laser, strobe0, strobe1; std::uint32_t exposure; std::uint16_t gain; int groups; };
    const Condition conditions[] = {
        {"scan", false, 100, 6000, 0, 4400, 120, scan_groups},   // as the app scans
        {"dark", false, 0, 0, 0, 4400, 120, 3},                  // black level
        {"strobe_only", false, 0, 6000, 0, 4400, 120, 5},        // IR ring light: markers
        {"projector_only", false, 100, 0, 0, 4400, 120, 5},      // speckle
        {"scan_exposure_8800", false, 100, 6000, 0, 8800, 120, 5},
        {"texture_white", true, 0, 6000, 9000, 4400, 120, 5},    // RGB with the white LEDs
    };
    for (const auto& c : conditions) {
        const fs::path dir = root / label / c.name;
        fs::create_directories(dir);
        (void)dev->set_exposure(0, c.exposure);
        for (int sensor = 0; sensor < 2; ++sensor) (void)dev->set_gain(sensor, c.gain);
        (void)dev->set_laser_percent(c.laser);
        (void)dev->set_strobe(0, c.strobe0);
        (void)dev->set_strobe(1, c.strobe1);
        (void)(c.texture ? dev->configure_texture_mode(100000) : dev->configure_scan_mode(68000));
        std::mutex m;
        std::vector<usb::FrameGroup> groups;
        int skipped = 0;
        (void)dev->start_stream([&](usb::FrameGroup&& g) {
            std::lock_guard lock(m);
            if (skipped < 3) {  // let the new settings settle
                ++skipped;
                return;
            }
            if (static_cast<int>(groups.size()) < c.groups) groups.push_back(std::move(g));
        });
        for (int i = 0; i < 400; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            std::lock_guard lock(m);
            if (static_cast<int>(groups.size()) >= c.groups) break;
        }
        (void)dev->set_trigger(0, 0);
        (void)dev->set_laser_percent(0);
        (void)dev->set_strobe(0, 0);
        (void)dev->set_strobe(1, 0);
        dev->stop_stream();
        std::ofstream csv(dir / "frames.csv");
        csv << "group,frame_id,sensor,subfield,timestamp_us\n";
        std::lock_guard lock(m);
        for (std::size_t i = 0; i < groups.size(); ++i)
            for (int sensor = 0; sensor < 3; ++sensor) {
                const auto& f = groups[i].sensors[static_cast<std::size_t>(sensor)];
                if (!f) continue;
                write_pgm(dir / std::format("g{:03}_s{}.pgm", i, sensor), f->pixels);
                csv << i << ',' << groups[i].frame_id << ',' << sensor << ',' << int(f->subfield) << ',' << f->timestamp << '\n';
            }
        std::ofstream(dir / "settings.txt") << std::format("mode {}\nlaser_percent {}\nstrobe_route0 {}\nstrobe_route1 {}\nexposure {}\ngain {}\n",
                                                          c.texture ? "texture (1 IR + RGB, 100 ms)" : "scan (3 IR, 68 ms)", c.laser, c.strobe0,
                                                          c.strobe1, c.exposure, c.gain);
        std::println("  {:20} {} groups", c.name, groups.size());
    }
    std::println("saved {}", (root / label).string());
    return 0;
}

// Is the calibration still right for these frames? Matches markers / board dots in raw pairs with the
// given calibration, then fits small corrections to the rig (right-camera rotation; + baseline direction;
// + right focal scale) that minimise the rectified row differences, and reports what is left.
// Directories hold gNNN_s0.pgm / gNNN_s1.pgm or imageLeftK.pgm / imageRightK.pgm pairs.
int rig_fit(std::span<char*> args) {
    namespace fs = std::filesystem;
    if (args.size() < 2) return usage();
    auto cal = load_calibration(args[0]);
    if (!cal) {
        std::println(stderr, "{}", cal.error().message);
        return 1;
    }
    const RigCalibration base = cal->rig();
    struct Match { Vec2 left, right; };
    std::vector<Match> matches;
    {
        const auto rect = calib::compute_rectification(base);
        const markers::MarkerStereo ms(base, rect, {});
        for (std::size_t a = 1; a < args.size(); ++a) {
            if (std::string_view(args[a]).starts_with("--")) continue;
            std::vector<std::pair<fs::path, fs::path>> pairs;
            for (const auto& e : fs::directory_iterator(args[a])) {
                const auto name = e.path().filename().string();
                if (name.ends_with("_s0.pgm")) pairs.emplace_back(e.path(), e.path().parent_path() / (name.substr(0, name.size() - 7) + "_s1.pgm"));
                if (name.starts_with("imageLeft") && name.ends_with(".pgm"))
                    pairs.emplace_back(e.path(), e.path().parent_path() / ("imageRight" + name.substr(9)));
            }
            int n = 0;
            for (const auto& [lp, rp] : pairs) {
                auto l = read_pgm(lp), r = read_pgm(rp);
                if (!l || !r) continue;
                const auto el = markers::detect_markers(l->view()), er = markers::detect_markers(r->view());
                for (const auto& e : el) {
                    const Vec2 ra = ms.rectify_left(e.center);
                    const markers::Ellipse* best = nullptr;
                    double bd = 12;  // true row errors stay under ~7 px; wider gates pick neighbouring board rows
                    for (const auto& f : er) {
                        const Vec2 rb = ms.rectify_right(f.center);
                        if (rb.x() < ra.x() && std::abs(rb.y() - ra.y()) < bd) bd = std::abs(rb.y() - ra.y()), best = &f;
                    }
                    if (best) matches.push_back({e.center, best->center}), ++n;
                }
            }
            std::println("{}: {} pairs, {} matched points", args[a], pairs.size(), n);
        }
    }
    if (matches.size() < 10) {
        std::println(stderr, "too few matched points");
        return 1;
    }
    {   // Trim mismatches: fit dy = a + b x + c y (rectified) to all points, drop those > 3 sigma, repeat.
        const auto rect = calib::compute_rectification(base);
        const markers::MarkerStereo ms(base, rect, {});
        for (int pass = 0; pass < 3; ++pass) {
            Eigen::MatrixXd A(static_cast<Eigen::Index>(matches.size()), 3);
            Eigen::VectorXd d(static_cast<Eigen::Index>(matches.size()));
            for (std::size_t i = 0; i < matches.size(); ++i) {
                const Vec2 l = ms.rectify_left(matches[i].left);
                A.row(static_cast<Eigen::Index>(i)) << 1, l.x(), l.y();
                d(static_cast<Eigen::Index>(i)) = ms.rectify_right(matches[i].right).y() - l.y();
            }
            const Eigen::Vector3d c = A.colPivHouseholderQr().solve(d);
            const Eigen::VectorXd r = d - A * c;
            const double sigma = std::sqrt(r.squaredNorm() / static_cast<double>(r.size()));
            std::vector<Match> kept;
            for (std::size_t i = 0; i < matches.size(); ++i)
                if (std::abs(r(static_cast<Eigen::Index>(i))) < std::max(3 * sigma, 0.3)) kept.push_back(matches[i]);
            if (kept.size() == matches.size()) break;
            std::println("  trimmed {} mismatched points", matches.size() - kept.size());
            matches = std::move(kept);
        }
    }
    // Parameters: [0..2] right-camera rotation (rad, applied in the right camera frame), [3..4] baseline
    // direction (y, z components added to the translation, mm), [5] right focal scale - 1.
    auto rig_of = [&](const std::array<double, 6>& q) {
        RigCalibration rig = base;
        const Mat3 dR = (Eigen::AngleAxisd(q[0], Vec3::UnitX()) * Eigen::AngleAxisd(q[1], Vec3::UnitY()) * Eigen::AngleAxisd(q[2], Vec3::UnitZ())).toRotationMatrix();
        rig.T_right_left.linear() = dR * base.T_right_left.linear();
        rig.T_right_left.translation() = dR * base.T_right_left.translation() + Vec3(0, q[3], q[4]);
        rig.right.fx *= 1 + q[5];
        rig.right.fy *= 1 + q[5];
        return rig;
    };
    auto cost = [&](const std::array<double, 6>& q) {
        const RigCalibration rig = rig_of(q);
        const auto rect = calib::compute_rectification(rig);
        const markers::MarkerStereo ms(rig, rect, {});
        double ss = 0;
        for (const auto& m : matches) {
            const double d = ms.rectify_right(m.right).y() - ms.rectify_left(m.left).y();
            ss += d * d;
        }
        return std::sqrt(ss / static_cast<double>(matches.size()));
    };
    // Nelder-Mead over the parameters a model frees.
    auto fit = [&](std::vector<int> free) {
        std::array<double, 6> q{};
        const std::size_t n = free.size();
        const double steps[6] = {1e-3, 1e-3, 1e-3, 0.5, 0.5, 1e-3};
        std::vector<std::array<double, 6>> simplex(n + 1, q);
        for (std::size_t i = 0; i < n; ++i) simplex[i + 1][static_cast<std::size_t>(free[i])] += steps[free[i]];
        std::vector<double> f(n + 1);
        for (std::size_t i = 0; i <= n; ++i) f[i] = cost(simplex[i]);
        for (int it = 0; it < 600; ++it) {
            std::vector<std::size_t> o(n + 1);
            std::iota(o.begin(), o.end(), 0);
            std::ranges::sort(o, [&](std::size_t a, std::size_t b) { return f[a] < f[b]; });
            const auto best = simplex[o[0]];
            const auto worst = simplex[o[n]];
            std::array<double, 6> c{};
            for (std::size_t i = 0; i < n; ++i)
                for (int k : free) c[static_cast<std::size_t>(k)] += simplex[o[i]][static_cast<std::size_t>(k)] / static_cast<double>(n);
            auto along = [&](double t) {
                std::array<double, 6> p = best;
                for (int k : free) p[static_cast<std::size_t>(k)] = c[static_cast<std::size_t>(k)] + t * (worst[static_cast<std::size_t>(k)] - c[static_cast<std::size_t>(k)]);
                return p;
            };
            const auto xr = along(-1);
            const double fr = cost(xr);
            if (fr < f[o[0]]) {
                const auto xe = along(-2);
                const double fe = cost(xe);
                if (fe < fr) simplex[o[n]] = xe, f[o[n]] = fe;
                else simplex[o[n]] = xr, f[o[n]] = fr;
            } else if (fr < f[o[n - 1]]) {
                simplex[o[n]] = xr, f[o[n]] = fr;
            } else {
                const auto xc = along(0.5);
                const double fc = cost(xc);
                if (fc < f[o[n]]) {
                    simplex[o[n]] = xc, f[o[n]] = fc;
                } else {
                    for (std::size_t i = 1; i <= n; ++i) {
                        for (int k : free)
                            simplex[o[i]][static_cast<std::size_t>(k)] = best[static_cast<std::size_t>(k)] + 0.5 * (simplex[o[i]][static_cast<std::size_t>(k)] - best[static_cast<std::size_t>(k)]);
                        f[o[i]] = cost(simplex[o[i]]);
                    }
                }
            }
        }
        const auto it = std::ranges::min_element(f);
        return std::pair{simplex[static_cast<std::size_t>(it - f.begin())], *it};
    };
    // Gauss-Newton on the per-point row differences (numeric Jacobian), from the Nelder-Mead result.
    auto refine = [&](std::array<double, 6> q, const std::vector<int>& free) {
        auto residuals = [&](const std::array<double, 6>& p) {
            const RigCalibration rig = rig_of(p);
            const auto rect = calib::compute_rectification(rig);
            const markers::MarkerStereo ms(rig, rect, {});
            Eigen::VectorXd r(static_cast<Eigen::Index>(matches.size()));
            for (std::size_t i = 0; i < matches.size(); ++i)
                r(static_cast<Eigen::Index>(i)) = ms.rectify_right(matches[i].right).y() - ms.rectify_left(matches[i].left).y();
            return r;
        };
        for (int it = 0; it < 15; ++it) {
            const Eigen::VectorXd r0 = residuals(q);
            Eigen::MatrixXd J(r0.size(), static_cast<Eigen::Index>(free.size()));
            for (std::size_t k = 0; k < free.size(); ++k) {
                auto qp = q;
                const double h = free[k] >= 3 && free[k] <= 4 ? 1e-3 : 1e-6;
                qp[static_cast<std::size_t>(free[k])] += h;
                J.col(static_cast<Eigen::Index>(k)) = (residuals(qp) - r0) / h;
            }
            const Eigen::VectorXd step = -(J.transpose() * J + 1e-9 * Eigen::MatrixXd::Identity(J.cols(), J.cols())).ldlt().solve(J.transpose() * r0);
            for (std::size_t k = 0; k < free.size(); ++k) q[static_cast<std::size_t>(free[k])] += step(static_cast<Eigen::Index>(k));
        }
        return q;
    };
    std::println("{} matched points; row rms with the calibration as is: {:.3f} px", matches.size(), cost({}));
    // Frame-formation models: an affine warp of one camera's raw pixel coordinates (before undistortion),
    // with the rig as calibrated. Gauss-Newton on the row residuals with a little damping (the warp's x
    // terms are only weakly observable from rows).
    {
        const auto rect = calib::compute_rectification(base);
        const markers::MarkerStereo ms(base, rect, {});
        for (const int cam : {1, 0}) {
            Eigen::Matrix<double, 6, 1> w = Eigen::Matrix<double, 6, 1>::Zero();  // x' = x + w0 + w1 (x-cx) + w2 (y-cy); y' = y + w3 + w4 (x-cx) + w5 (y-cy)
            const double cx = 640, cy = 512;
            auto warp = [&](const Vec2& p, const Eigen::Matrix<double, 6, 1>& q) {
                const double dx = p.x() - cx, dy = p.y() - cy;
                return Vec2(p.x() + q(0) + q(1) * dx + q(2) * dy, p.y() + q(3) + q(4) * dx + q(5) * dy);
            };
            auto residuals = [&](const Eigen::Matrix<double, 6, 1>& q) {
                Eigen::VectorXd r(static_cast<Eigen::Index>(matches.size()));
                for (std::size_t i = 0; i < matches.size(); ++i) {
                    const Vec2 L = cam == 0 ? warp(matches[i].left, q) : matches[i].left;
                    const Vec2 R = cam == 1 ? warp(matches[i].right, q) : matches[i].right;
                    r(static_cast<Eigen::Index>(i)) = ms.rectify_right(R).y() - ms.rectify_left(L).y();
                }
                return r;
            };
            for (int it = 0; it < 20; ++it) {
                const Eigen::VectorXd r0 = residuals(w);
                Eigen::MatrixXd J(r0.size(), 6);
                for (int k = 0; k < 6; ++k) {
                    auto q = w;
                    const double h = (k == 0 || k == 3) ? 1e-3 : 1e-6;
                    q(k) += h;
                    J.col(k) = (residuals(q) - r0) / h;
                }
                const Eigen::Matrix<double, 6, 1> step = -(J.transpose() * J + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(J.transpose() * r0);
                w += step;
            }
            const Eigen::VectorXd r = residuals(w);
            std::println("  affine warp of the {} raw image: row rms {:.3f} px  (shift ({:+.2f}, {:+.2f}) px, x' scale {:+.5f} shear {:+.5f}, y' shear {:+.5f} scale {:+.5f})",
                         cam == 0 ? "left" : "right", std::sqrt(r.squaredNorm() / static_cast<double>(r.size())), w(0), w(3), w(1), w(2), w(4), w(5));
        }
    }
    const double deg = 180.0 / M_PI;
    for (const auto& [name, free] : {std::pair{"rotation of the right camera", std::vector<int>{0, 1, 2}},
                                     std::pair{"+ baseline direction", std::vector<int>{0, 1, 2, 3, 4}},
                                     std::pair{"+ right focal scale", std::vector<int>{0, 1, 2, 3, 4, 5}}}) {
        auto [q0, rms0] = fit(free);
        const auto q = refine(q0, free);
        const double rms = cost(q);
        std::println("  {:30} row rms {:.3f} px  (rx {:+.4f} ry {:+.4f} rz {:+.4f} deg, ty {:+.3f} tz {:+.3f} mm, f x{:.5f})", name, rms,
                     q[0] * deg, q[1] * deg, q[2] * deg, q[3], q[4], 1 + q[5]);
    }
    return 0;
}

// Does each camera's image of the flat calibration board fit its own intrinsics? The dots, undistorted
// with the camera's calibration, must lie on an exact homography of the 5 x 8 grid; the residual (in
// pixels) says how far the image departs from the camera model, independently of the other camera.
int board_check(std::span<char*> args) {
    namespace fs = std::filesystem;
    if (args.size() < 2) return usage();
    auto cal = load_calibration(args[0]);
    if (!cal) {
        std::println(stderr, "{}", cal.error().message);
        return 1;
    }
    const RigCalibration rig = cal->rig();
    auto homography = [](const std::vector<Vec2>& src, const std::vector<Vec2>& dst) {  // DLT, dst ~ H src
        Eigen::MatrixXd A(2 * src.size(), 9);
        for (std::size_t i = 0; i < src.size(); ++i) {
            const double x = src[i].x(), y = src[i].y(), u = dst[i].x(), v = dst[i].y();
            A.row(static_cast<Eigen::Index>(2 * i)) << -x, -y, -1, 0, 0, 0, u * x, u * y, u;
            A.row(static_cast<Eigen::Index>(2 * i + 1)) << 0, 0, 0, -x, -y, -1, v * x, v * y, v;
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
        const Eigen::VectorXd h = svd.matrixV().col(8);
        Mat3 H;
        H << h(0), h(1), h(2), h(3), h(4), h(5), h(6), h(7), h(8);
        return H;
    };
    auto apply = [](const Mat3& H, const Vec2& p) {
        const Vec3 q = H * Vec3(p.x(), p.y(), 1);
        return Vec2(q.x() / q.z(), q.y() / q.z());
    };
    // Grid positions of the board's dots from its four large ones (three in grid row 1 at columns 2, 4, 5,
    // one between columns 3 and 4 of row 3), which also fix the orientation. Returns grid -> undistorted normalised.
    struct GridView { std::vector<Vec2> grid, normalised, raw; double residual_px = 0; };
    auto view_of = [&](const fs::path& path, const CameraModel& cam) -> std::optional<GridView> {
        auto img = read_pgm(path);
        if (!img) return std::nullopt;
        const auto dots = markers::detect_markers(img->view());
        if (dots.size() < 30) return std::nullopt;
        std::vector<double> sizes;
        for (const auto& d : dots) sizes.push_back(d.a);
        std::ranges::sort(sizes);
        const double med = sizes[sizes.size() / 2];
        std::vector<Vec2> n, big, raw;
        for (const auto& d : dots) {
            raw.push_back(d.center);
            n.push_back(calib::undistort_to_normalized(cam, d.center));
            if (d.a > 1.5 * med) big.push_back(n.back());
        }
        if (big.size() != 4) return std::nullopt;
        // The lone large dot is the one farthest from the line through the other three.
        std::size_t lone = 0;
        double worst = -1;
        for (std::size_t i = 0; i < 4; ++i) {
            std::vector<Vec2> o;
            for (std::size_t j = 0; j < 4; ++j)
                if (j != i) o.push_back(big[j]);
            const Vec2 d = (o[2] - o[0]).normalized();
            const double off = std::abs(d.x() * (o[1] - o[0]).y() - d.y() * (o[1] - o[0]).x());
            if (off < worst || worst < 0) worst = off, lone = i;  // the three others are collinear
        }
        std::vector<Vec2> row;
        for (std::size_t j = 0; j < 4; ++j)
            if (j != lone) row.push_back(big[j]);
        // Columns 2, 4, 5: the endpoints are the farthest-apart pair; column 2 is the one farther from the middle.
        std::size_t e1 = 0, e2 = 1;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = i + 1; j < 3; ++j)
                if ((row[i] - row[j]).norm() > (row[e1] - row[e2]).norm()) e1 = i, e2 = j;
        const Vec2 mid = row[3 - e1 - e2];
        const bool first_is_col2 = (row[e1] - mid).norm() > (row[e2] - mid).norm();
        const std::vector<Vec2> r1 = {first_is_col2 ? row[e1] : row[e2], mid, first_is_col2 ? row[e2] : row[e1]};
        // Initial map: affine from the four large dots (three are collinear, so no homography yet).
        Mat3 H = Mat3::Identity();
        {
            const Vec2 g[4] = {{2, 1}, {4, 1}, {5, 1}, {3.5, 3}};  // (the lone one sits between columns 3 and 4)
            const Vec2 m[4] = {r1[0], r1[1], r1[2], big[lone]};
            Eigen::Matrix<double, 8, 6> A = Eigen::Matrix<double, 8, 6>::Zero();
            Eigen::Matrix<double, 8, 1> b;
            for (int i = 0; i < 4; ++i) {
                A.row(2 * i) << g[i].x(), g[i].y(), 1, 0, 0, 0;
                A.row(2 * i + 1) << 0, 0, 0, g[i].x(), g[i].y(), 1;
                b(2 * i) = m[i].x();
                b(2 * i + 1) = m[i].y();
            }
            const Eigen::Matrix<double, 6, 1> a = A.colPivHouseholderQr().solve(b);
            H << a(0), a(1), a(2), a(3), a(4), a(5), 0, 0, 1;
        }
        GridView v;
        for (int iter = 0; iter < 3; ++iter) {
            v.grid.clear();
            v.normalised.clear();
            v.raw.clear();
            for (int gy = 0; gy < 5; ++gy)
                for (int gx = 0; gx < 8; ++gx) {
                    const Vec2 pred = apply(H, Vec2(gx, gy));
                    std::size_t best = n.size();
                    double bd = 1e9;
                    for (std::size_t i = 0; i < n.size(); ++i)
                        if ((n[i] - pred).norm() < bd) bd = (n[i] - pred).norm(), best = i;
                    if (best < n.size() && bd * cam.fx < 12) v.grid.emplace_back(gx, gy), v.normalised.push_back(n[best]), v.raw.push_back(raw[best]);
                }
            if (v.grid.size() >= 8) H = homography(v.grid, v.normalised);
        }
        double ss = 0;
        for (std::size_t i = 0; i < v.grid.size(); ++i) ss += (apply(H, v.grid[i]) - v.normalised[i]).squaredNorm() * cam.fx * cam.fx;
        v.residual_px = std::sqrt(ss / static_cast<double>(std::max<std::size_t>(v.grid.size(), 1)));
        if (v.grid.size() < 20) return std::nullopt;
        return v;
    };
    // Board pose in a camera from the normalised homography (grid in pitch units), refined by Gauss-Newton
    // on the reprojection error.
    constexpr double kPitch = 27.89;  // mm (EXStar's own analysis of this board)
    auto pose_of = [&](const GridView& v) {
        const Mat3 H = homography(v.grid, v.normalised);
        const double lambda = 1.0 / H.col(0).norm();
        Mat3 R;
        R.col(0) = lambda * H.col(0);
        R.col(1) = lambda * H.col(1);
        R.col(2) = R.col(0).cross(R.col(1));
        Eigen::JacobiSVD<Mat3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0) R = -R;
        Vec3 t = lambda * H.col(2) * kPitch;
        if (t.z() < 0) R.col(0) = -R.col(0), R.col(1) = -R.col(1), t = -t;
        SE3 T = SE3::Identity();  // board (mm) -> camera
        T.linear() = R;
        T.translation() = t;
        for (int it = 0; it < 10; ++it) {  // Gauss-Newton, 6 dof, numeric Jacobian
            Eigen::MatrixXd J(2 * v.grid.size(), 6);
            Eigen::VectorXd e(2 * v.grid.size());
            auto residuals = [&](const SE3& P, Eigen::VectorXd& out) {
                for (std::size_t i = 0; i < v.grid.size(); ++i) {
                    const Vec3 c = P * Vec3(v.grid[i].x() * kPitch, v.grid[i].y() * kPitch, 0);
                    out(static_cast<Eigen::Index>(2 * i)) = c.x() / c.z() - v.normalised[i].x();
                    out(static_cast<Eigen::Index>(2 * i + 1)) = c.y() / c.z() - v.normalised[i].y();
                }
            };
            residuals(T, e);
            for (int k = 0; k < 6; ++k) {
                Eigen::Matrix<double, 6, 1> d = Eigen::Matrix<double, 6, 1>::Zero();
                d(k) = k < 3 ? 1e-6 : 1e-4;
                SE3 P = T;
                P.linear() = Eigen::AngleAxisd(d.head<3>().norm() > 0 ? d.head<3>().norm() : 0, d.head<3>().norm() > 0 ? Vec3(d.head<3>().normalized()) : Vec3::UnitX()).toRotationMatrix() * T.linear();
                P.translation() = T.translation() + d.tail<3>();
                Eigen::VectorXd e2(e.size());
                residuals(P, e2);
                J.col(k) = (e2 - e) / d(k);
            }
            const Eigen::Matrix<double, 6, 1> step = -(J.transpose() * J).ldlt().solve(J.transpose() * e);
            const Vec3 w = step.head<3>();
            if (w.norm() > 0) T.linear() = Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix() * T.linear();
            T.translation() += step.tail<3>();
        }
        return T;
    };
    // Focal scale of a camera from one board view: board pose + a common scale of fx, fy (the tilt of the
    // board separates focal length from distance). Returns the scale relative to the calibration.
    auto focal_scale_of = [&](const GridView& v) {
        SE3 T = pose_of(v);
        double sc = 1.0;
        for (int it = 0; it < 20; ++it) {
            auto residuals = [&](const SE3& P, double f, Eigen::VectorXd& out) {
                for (std::size_t i = 0; i < v.grid.size(); ++i) {
                    const Vec3 c = P * Vec3(v.grid[i].x() * kPitch, v.grid[i].y() * kPitch, 0);
                    out(static_cast<Eigen::Index>(2 * i)) = f * c.x() / c.z() - v.normalised[i].x();
                    out(static_cast<Eigen::Index>(2 * i + 1)) = f * c.y() / c.z() - v.normalised[i].y();
                }
            };
            Eigen::VectorXd e(2 * static_cast<Eigen::Index>(v.grid.size()));
            residuals(T, sc, e);
            Eigen::MatrixXd J(e.size(), 7);
            for (int k = 0; k < 7; ++k) {
                SE3 P = T;
                double f = sc;
                const double h = k < 3 ? 1e-6 : k < 6 ? 1e-4 : 1e-6;
                if (k < 3) P.linear() = Eigen::AngleAxisd(h, Vec3::Unit(k)).toRotationMatrix() * T.linear();
                else if (k < 6) P.translation()(k - 3) += h;
                else f += h;
                Eigen::VectorXd e2(e.size());
                residuals(P, f, e2);
                J.col(k) = (e2 - e) / h;
            }
            const Eigen::Matrix<double, 7, 1> step = -(J.transpose() * J).ldlt().solve(J.transpose() * e);
            const Vec3 w = step.head<3>();
            if (w.norm() > 0) T.linear() = Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix() * T.linear();
            T.translation() += step.segment<3>(3);
            sc += step(6);
        }
        return sc;
    };
    // (Normalised coordinates scale inversely with the focal length: the camera's true focal length is the
    // calibrated one divided by the fitted factor on the normalised image.)
    const double deg = 180.0 / M_PI;
    const SE3 T_cal = rig.T_right_left;
    const bool ba = has_flag(args, "--ba");
    std::vector<optim::BoardView> views;
    std::vector<std::string> set_names;
    for (std::size_t a = 1; a < args.size(); ++a) {
        if (std::string_view(args[a]).starts_with("--") || std::string_view(args[a - 1]) == "--save-rig") continue;
        set_names.emplace_back(args[a]);
        std::vector<fs::path> lefts;
        for (const auto& e : fs::directory_iterator(args[a])) {
            const auto name = e.path().filename().string();
            if (name.ends_with("_s0.pgm") || (name.starts_with("imageLeft") && name.ends_with(".pgm"))) lefts.push_back(e.path());
        }
        std::ranges::sort(lefts);
        for (const auto& lp : lefts) {
            const auto name = lp.filename().string();
            const fs::path rp = lp.parent_path() / (name.starts_with("imageLeft") ? "imageRight" + name.substr(9) : name.substr(0, name.size() - 7) + "_s1.pgm");
            const auto l = view_of(lp, rig.left), r = view_of(rp, rig.right);
            std::print("{:52} left {:>13}  right {:>13}", lp.string(), l ? std::format("{:.3f} px ({})", l->residual_px, l->grid.size()) : "-",
                       r ? std::format("{:.3f} px ({})", r->residual_px, r->grid.size()) : "-");
            if (l && r && ba) {
                optim::BoardView bv;
                bv.set = static_cast<int>(set_names.size()) - 1;
                bv.T_left_board = pose_of(*l);
                for (std::size_t i = 0; i < l->grid.size(); ++i)
                    for (std::size_t j = 0; j < r->grid.size(); ++j)
                        if (l->grid[i] == r->grid[j]) {
                            bv.board.emplace_back(l->grid[i].x() * kPitch, l->grid[i].y() * kPitch, 0);
                            bv.left.push_back(l->raw[i]);
                            bv.right.push_back(r->raw[j]);
                        }
                if (bv.board.size() >= 20) views.push_back(std::move(bv));
            }
            if (l && r) {
                const SE3 T_rl = pose_of(*r) * pose_of(*l).inverse();  // left camera -> right camera
                const SE3 D = T_cal.inverse() * T_rl;                 // difference, in the left camera frame
                const Eigen::AngleAxisd aa(D.linear());
                const Vec3 axis = aa.axis() * aa.angle() * deg;
                std::print("  vs calibration: rotation ({:+.3f}, {:+.3f}, {:+.3f}) deg, translation ({:+.2f}, {:+.2f}, {:+.2f}) mm, baseline {:.2f} mm",
                           axis.x(), axis.y(), axis.z(), D.translation().x(), D.translation().y(), D.translation().z(),
                           T_rl.translation().norm());
                if (has_flag(args, "--focal"))
                    std::print("; focal scale left {:.4f} right {:.4f}", 1.0 / focal_scale_of(*l), 1.0 / focal_scale_of(*r));
            }
            std::println("");
        }
    }
    if (!ba || views.empty()) return 0;
    auto report = [&](const char* title, const optim::StereoCalibResult& r) {
        std::print("  {:34} rms", title);
        for (std::size_t si = 0; si < r.rms_by_set.size(); ++si) std::print(" [{}] {:.3f}", si, r.rms_by_set[si]);
        const SE3 D = rig.T_right_left.inverse() * r.rig.T_right_left;
        const Eigen::AngleAxisd aa(D.linear());
        const Vec3 rv = aa.axis() * aa.angle() * deg;
        std::println(" px | left f {:+.2f}/{:+.2f} c {:+.2f}/{:+.2f}, right f {:+.2f}/{:+.2f} c {:+.2f}/{:+.2f} px, rig rot ({:+.3f}, {:+.3f}, {:+.3f}) deg t ({:+.2f}, {:+.2f}, {:+.2f}) mm",
                     r.rig.left.fx - rig.left.fx, r.rig.left.fy - rig.left.fy, r.rig.left.cx - rig.left.cx, r.rig.left.cy - rig.left.cy,
                     r.rig.right.fx - rig.right.fx, r.rig.right.fy - rig.right.fy, r.rig.right.cx - rig.right.cx, r.rig.right.cy - rig.right.cy,
                     rv.x(), rv.y(), rv.z(), D.translation().x(), D.translation().y(), D.translation().z());
    };
    std::println("== stereo bundle adjustment ({} views; sets:{}) ==", views.size(), [&] {
        std::string t;
        for (std::size_t i = 0; i < set_names.size(); ++i) t += std::format(" [{}] {}", i, set_names[i]);
        return t;
    }());
    optim::StereoCalibOptions fixed;
    fixed.free_intrinsics = fixed.free_rig = false;
    report("calibration as stored (poses only)", optim::refine_stereo_calibration(rig, views, fixed));
    report("joint: intrinsics + rig free", optim::refine_stereo_calibration(rig, views, {}));
    optim::StereoCalibOptions with_dist;
    with_dist.free_distortion = true;
    report("joint: + distortion free", optim::refine_stereo_calibration(rig, views, with_dist));
    for (std::size_t si = 0; si < set_names.size(); ++si) {
        std::vector<optim::BoardView> one;
        for (const auto& v : views)
            if (v.set == static_cast<int>(si)) one.push_back(v), one.back().set = 0;
        if (one.size() < 2) continue;
        const auto r = optim::refine_stereo_calibration(rig, one, {});
        report(std::format("set [{}] alone: intrinsics + rig free", si).c_str(), r);
        if (const char* out = arg_str(args, "--save-rig"); out && si + 1 == set_names.size()) {  // the last set's model
            std::ofstream f(out);
            for (const auto* m : {&r.rig.left, &r.rig.right}) {
                f << std::format("{:.9g} {:.9g} {:.9g} {:.9g} {:.9g}", m->fx, m->fy, m->cx, m->cy, m->skew);
                for (const double d : m->dist) f << std::format(" {:.9g}", d);
                f << '\n';
            }
            for (int i = 0; i < 9; ++i) f << std::format("{:.12g} ", r.rig.T_right_left.linear()(i / 3, i % 3));
            f << '\n';
            for (int i = 0; i < 3; ++i) f << std::format("{:.9g} ", r.rig.T_right_left.translation()(i));
            f << '\n';
            std::println("  saved the set [{}] model to {} (use as rig:{})", si, out, out);
        }
    }
    return 0;
}

// Board poses of saved calibration images (EXStar's imageLeftN / our gNNN_s0): dots found, grid extent
// and the board's pose in each camera. --pad N searches N extra grid rows/columns around the 8 x 5 grid.
int board_poses(std::span<char*> args) {
    namespace fs = std::filesystem;
    if (args.size() < 2) return usage();
    auto cal = load_calibration(args[0]);
    if (!cal) {
        std::println(stderr, "{}", cal.error().message);
        return 1;
    }
    const RigCalibration rig = cal->rig();
    const int pad = static_cast<int>(arg_int(args, "--pad", 0));
    calibrate::BoardSpec spec;
    spec.cols += 2 * pad, spec.rows += 2 * pad;
    for (auto& l : spec.large) l += Vec2(pad, pad);
    std::map<std::pair<int, int>, int> seen;
    const double deg = 180.0 / M_PI;
    for (std::size_t a = 1; a < args.size(); ++a) {
        if (std::string_view(args[a]).starts_with("--") || std::string_view(args[a - 1]).starts_with("--")) continue;
        std::vector<fs::path> lefts;
        for (const auto& e : fs::directory_iterator(args[a])) {
            const auto name = e.path().filename().string();
            if (name.ends_with("_s0.pgm") || (name.starts_with("imageLeft") && name.ends_with(".pgm"))) lefts.push_back(e.path());
        }
        std::ranges::sort(lefts, [](const fs::path& x, const fs::path& y) {
            auto num = [](const fs::path& p) { std::string n = p.stem().string(); n.erase(std::remove_if(n.begin(), n.end(), [](char c) { return !std::isdigit(c); }), n.end()); return n.empty() ? 0 : std::stoi(n); };
            return num(x) < num(y);
        });
        for (const auto& lp : lefts) {
            const auto name = lp.filename().string();
            const fs::path rp = lp.parent_path() / (name.starts_with("imageLeft") ? "imageRight" + name.substr(9) : name.substr(0, name.size() - 7) + "_s1.pgm");
            std::print("{:24}", name);
            for (int cam = 0; cam < 2; ++cam) {
                const auto img = read_pgm(cam == 0 ? lp : rp);
                const auto d = img ? calibrate::detect_board(img->view(), spec) : std::nullopt;
                if (!d) {
                    std::print("  {:>58}", "-");
                    continue;
                }
                int c0 = 99, c1 = -99, r0 = 99, r1 = -99;
                for (const auto& g : d->grid) {
                    c0 = std::min(c0, int(g.x()) - pad), c1 = std::max(c1, int(g.x()) - pad), r0 = std::min(r0, int(g.y()) - pad), r1 = std::max(r1, int(g.y()) - pad);
                    ++seen[{int(g.x()) - pad, int(g.y()) - pad}];
                }
                const auto pose = calibrate::board_pose(*d, cam == 0 ? rig.left : rig.right, spec);
                if (!pose) continue;
                const Mat3 R = pose->T_cam_board.linear();
                const Vec3 c = pose->T_cam_board * spec.centre();
                const Vec3 n = R.col(2);
                std::print("  {:2} dots c{:+d}..{:+d} r{:+d}..{:+d} res {:.2f} rms {:.2f} | z {:5.1f} tilt {:+5.1f} {:+5.1f} roll {:+6.1f}", d->size(), c0, c1, r0, r1,
                           d->residual_px, pose->rms_px, c.z(), std::atan2(n.y(), n.z()) * deg, std::atan2(n.x(), n.z()) * deg,
                           std::atan2(R(1, 0), R(0, 0)) * deg);
            }
            std::println("");
        }
    }
    if (pad > 0) {
        std::println("grid positions seen (column, row: count):");
        for (const auto& [k, v] : seen) std::print(" ({},{}):{}", k.first, k.second, v);
        std::println("");
    }
    return 0;
}

// Calibrates the IR pair from a folder of board captures (einstar-calibrate's or EXStar's imageLeftN /
// imageRightN), independently of any stored calibration, and compares the result with a reference
// calibration (e.g. the scanner's flash, which EXStar made) evaluated on the same captures.
int calib_solve(std::span<char*> args) {
    if (args.empty()) return usage();
    const auto loaded = calibrate::load_captures(args[0]);
    if (!loaded) {
        std::println(stderr, "{}", loaded.error().message);
        return 1;
    }
    std::optional<RigCalibration> reference;
    if (const char* ref = arg_str(args, "--reference")) {
        auto cal = load_calibration(ref);
        if (!cal) {
            std::println(stderr, "{}", cal.error().message);
            return 1;
        }
        reference = cal->rig();
    }
    calibrate::SolveOptions so;
    so.free_distortion = !has_flag(args, "--no-distortion");
    if (const char* from = arg_str(args, "--distortion-from")) {
        auto cal = load_calibration(from);
        if (!cal) {
            std::println(stderr, "{}", cal.error().message);
            return 1;
        }
        so.fixed_distortion = std::array{cal->left.model.dist, cal->right.model.dist};
    }
    Stopwatch sw;
    auto ours = calibrate::solve_stereo(loaded->captures, loaded->width, loaded->height, {}, so);
    if (!ours) {
        std::println(stderr, "solve failed: {}", ours.error().message);
        return 1;
    }
    std::println("solved in {:.0f} ms: {} views, {} dots ({} outliers dropped)", sw.elapsed_ms(), std::ranges::count_if(ours->views, &calibrate::ViewReport::used), ours->dots, ours->dropped);
    std::optional<calibrate::CalibrationReport> ref_report;
    if (reference) {
        if (auto r = calibrate::evaluate_calibration(*reference, loaded->captures)) ref_report = std::move(*r);
        else std::println(stderr, "reference: {}", r.error().message);
    }
    std::println("{:18} {:>4} {:>6} {:>13} | {:>8} {:>8}{}", "view", "dots", "dist", "tilt x/y", "rms", "row rms", ref_report ? " | reference rms / row rms" : "");
    for (std::size_t i = 0; i < ours->views.size(); ++i) {
        const auto& v = ours->views[i];
        const auto& c = loaded->captures[i];
        if (!v.used) {
            std::println("{:18} not used (left {} dots, right {})", v.name, c.left.size(), c.right.size());
            continue;
        }
        std::print("{:18} {:>4} {:>6.0f} {:>+6.1f} {:>+6.1f} | {:>8.3f} {:>8.3f}", v.name, v.dots, v.distance_mm, v.tilt_x_deg, v.tilt_y_deg, v.rms_px, v.row_rms_px);
        if (ref_report) std::print(" | {:>8.3f} {:>8.3f}", ref_report->views[i].rms_px, ref_report->views[i].row_rms_px);
        std::println("");
    }
    std::println("{:18} {:>4} {:>6} {:>13} | {:>8.3f} {:>8.3f}{}", "all", ours->dots, "", "", ours->rms_px, ours->row_rms_px,
                 ref_report ? std::format(" | {:>8.3f} {:>8.3f}", ref_report->rms_px, ref_report->row_rms_px) : "");
    auto camera_line = [](const char* name, const CameraModel& m) {
        std::println("  {:6} f {:9.3f} {:9.3f}  c {:8.3f} {:8.3f}  k {:+.5f} {:+.5f} {:+.6f} {:+.6f} {:+.5f}", name, m.fx, m.fy, m.cx, m.cy, m.dist[0], m.dist[1],
                     m.dist[2], m.dist[3], m.dist[4]);
    };
    auto rig_line = [](const RigCalibration& r) {
        const Eigen::AngleAxisd aa(r.T_right_left.linear());
        const Vec3 rv = aa.axis() * aa.angle() * 180.0 / M_PI;
        const Vec3 t = r.T_right_left.translation();
        std::println("  rig    rot ({:+.4f}, {:+.4f}, {:+.4f}) deg  t ({:+.3f}, {:+.3f}, {:+.3f}) mm  baseline {:.3f} mm", rv.x(), rv.y(), rv.z(), t.x(), t.y(), t.z(), r.baseline_mm());
    };
    std::println("ours:");
    camera_line("left", ours->rig.left);
    camera_line("right", ours->rig.right);
    rig_line(ours->rig);
    if (reference) {
        std::println("reference:");
        camera_line("left", reference->left);
        camera_line("right", reference->right);
        rig_line(*reference);
        const auto d = calibrate::compare_calibrations(*reference, ours->rig);
        std::println("ours - reference:");
        for (const auto& [name, c] : {std::pair{"left", d.left}, std::pair{"right", d.right}})
            std::println("  {:6} f {:+.3f} {:+.3f}  c {:+.3f} {:+.3f}  distortion alone up to {:.3f} px, ray mapping up to {:.3f} px", name, c.dfx, c.dfy, c.dcx,
                         c.dcy, c.distortion_px, c.mapping_px);
        std::println("  rig    rot ({:+.4f}, {:+.4f}, {:+.4f}) deg  t ({:+.3f}, {:+.3f}, {:+.3f}) mm  baseline {:+.3f} mm", d.rotation_deg.x(), d.rotation_deg.y(),
                     d.rotation_deg.z(), d.translation_mm.x(), d.translation_mm.y(), d.translation_mm.z(), d.baseline_mm);
    }
    if (const char* out = arg_str(args, "--save")) {
        calibrate::CalibrationFile f;
        f.rig = ours->rig;
        if (reference) f.rig.texture = reference->texture, f.rig.T_texture_left = reference->T_texture_left;
        f.source = std::format("einstar-cli calib-solve {}", args[0]);
        f.rms_px = ours->rms_px, f.row_rms_px = ours->row_rms_px;
        f.views = static_cast<int>(std::ranges::count_if(ours->views, &calibrate::ViewReport::used));
        if (auto r = calibrate::write_calibration_file(out, f); !r) {
            std::println(stderr, "{}", r.error().message);
            return 1;
        }
        std::println("saved {}", out);
    }
    return 0;
}

int fixture_pack(std::span<char*> args) {
    namespace fs = std::filesystem;
    const char* out_arg = arg_str(args, "--out");
    if (!out_arg) return usage();
    const fs::path out = out_arg;
    auto packed_size = [](const fs::path& dir) {
        std::uintmax_t n = 0;
        for (const auto& e : fs::directory_iterator(dir)) n += e.file_size();
        return static_cast<double>(n) / (1024.0 * 1024.0);
    };
    // Writes `tmp` into `dir` as `<name>.zst`.
    auto pack = [](const fs::path& tmp, const fs::path& dir) -> bool {
        if (auto r = fixtures::compress_file(tmp, dir / (tmp.filename().string() + ".zst")); !r) {
            std::println(stderr, "{}", r.error().message);
            return false;
        }
        return true;
    };
    const fs::path staging = fs::temp_directory_path() / std::format("einstar-fixture-pack-{}", ::getpid());
    fs::create_directories(staging);

    if (const char* src = arg_str(args, "--mustang")) {
        const fs::path dir = out / "mustang";
        fs::create_directories(dir);
        auto full = fixtures::ExstarProject::open(src);
        if (!full) {
            std::println(stderr, "{}", full.error().message);
            return 1;
        }
        const auto frames = fixtures::mustang_test_frames();
        if (auto r = fixtures::write_project_subset(src, frames, staging / "Project1"); !r) {
            std::println(stderr, "{}", r.error().message);
            return 1;
        }
        for (const char* ext : {".data_base", ".data_cm", ".ir_E10_prj"})
            if (!pack(staging / (std::string("Project1") + ext), dir)) return 1;
        std::ofstream manifest(dir / "manifest.txt");
        manifest << "# Excerpt of an EXStar recording (einstar-cli fixture-pack): the frames the tests read.\n"
                 << "# frame i of Project1 is the recording's frame frames[i].\n"
                 << "frame_count " << (*full)->frame_count() << "\nframes";
        for (const auto f : frames) manifest << ' ' << f;
        manifest << '\n';
        if (const char* stl = arg_str(args, "--stl")) {
            auto mesh = fixtures::load_stl(stl);
            if (!mesh) {
                std::println(stderr, "{}", mesh.error().message);
                return 1;
            }
            // Only the part of EXStar's mesh the compared frames see (plus a margin).
            Vec3f lo = Vec3f::Constant(1e30f), hi = Vec3f::Constant(-1e30f);
            for (const auto i : fixtures::mustang_mesh_frames()) {
                auto f = (*full)->read_frame(i);
                if (!f) continue;
                const Eigen::Matrix4f T = f->T_world_camera.matrix().cast<float>();
                for (const auto& p : fixtures::unproject(*f, 8)) {
                    const Vec3f w = (T * p.homogeneous()).head<3>();
                    lo = lo.cwiseMin(w);
                    hi = hi.cwiseMax(w);
                }
            }
            const auto cropped = fixtures::crop(*mesh, lo - Vec3f::Constant(10), hi + Vec3f::Constant(10));
            if (auto r = fixtures::write_stl(cropped, staging / "mesh.stl"); !r || !pack(staging / "mesh.stl", dir)) return 1;
            std::println("mesh: {} of {} triangles", cropped.vertices.size() / 3, mesh->vertices.size() / 3);
        }
        std::println("{}: {} of {} frames, {:.1f} MiB", dir.string(), frames.size(), (*full)->frame_count(), packed_size(dir));
    }

    if (const char* board = arg_str(args, "--board")) {
        const fs::path dir = out / "calibration_board";
        fs::create_directories(dir);
        int pairs = 0;
        for (int k = 1; k <= 25; ++k) {
            bool both = true;
            for (const char* side : {"Left", "Right"})
                both = both && fs::exists(fs::path(board) / std::format("image{}{}.bmp", side, k));
            if (!both) continue;
            for (const char* side : {"Left", "Right"}) {
                const auto name = std::format("image{}{}.bmp", side, k);
                if (auto r = fixtures::compress_file(fs::path(board) / name, dir / (name + ".zst")); !r) {
                    std::println(stderr, "{}", r.error().message);
                    return 1;
                }
            }
            ++pairs;
        }
        std::println("{}: {} image pairs, {:.1f} MiB", dir.string(), pairs, packed_size(dir));
    }
    fs::remove_all(staging);
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
    if (cmd == "calib-dump" && argc >= 3) return calib_dump_cmd(argv[2]);
    if (cmd == "hw-test") return hw_test(rest);
    if (cmd == "fixture-pack") return fixture_pack(rest);
    if (cmd == "hw-ui") return hw_ui(rest);
    if (cmd == "hw-lights") return hw_lights(rest);
    if (cmd == "markers-debug") return markers_debug(rest);
    if (cmd == "stereo-debug") return stereo_debug(rest);
    if (cmd == "hw-capture") return hw_capture(rest);
    if (cmd == "rig-fit") return rig_fit(rest);
    if (cmd == "board-check") return board_check(rest);
    if (cmd == "board-poses") return board_poses(rest);
    if (cmd == "calib-solve") return calib_solve(rest);
    if (cmd == "track-fixture" && argc >= 3) return track_fixture(argv[2], rest);
    if (cmd == "inspect" && argc >= 3) return inspect_cmd(argv[2]);
    if (cmd == "process" && argc >= 3) return process_cmd(argv[2], std::span<char*>(argv + 3, static_cast<std::size_t>(argc - 3)));
    return usage();
}
