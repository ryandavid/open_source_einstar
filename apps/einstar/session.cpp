#include "session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/log.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/synth/demo.hpp"

namespace einstar::app {
namespace {

constexpr const char* kExstarCalibrationCache = "/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/200x150";

// Emulated scene: a cluttered table-top the virtual scanner sweeps over.
struct EmulatorScene {
    synth::Scene scene;
    synth::Projector projector;
    RigCalibration rig;

    [[nodiscard]] SE3 pose(double t) const {
        // Slow figure-eight over the objects, aiming drifts gently (seconds -> left IR camera pose).
        const double a = 0.35 * t;
        const Vec3 eye(-90 + 120 * std::sin(a), -150 - 20 * std::sin(2 * a), -230 + 40 * std::cos(a));
        const Vec3 target(-80 + 40 * std::sin(a + 0.5), 30, 10 + 20 * std::cos(a));
        SE3 T = synth::look_at(eye, target);
        const double half_toe = 0.5 * rotation_angle(rig.T_right_left);
        T.linear() = T.linear() * Eigen::AngleAxisd(half_toe, Vec3::UnitY()).toRotationMatrix();
        return T;
    }
};

std::shared_ptr<EmulatorScene> make_scene(const RigCalibration& rig) {
    auto e = std::make_shared<EmulatorScene>();
    e->rig = rig;
    e->scene = synth::table_scene();
    e->scene.markers = synth::scatter_markers(60, 5, -260, 120, -150, 170);
    e->projector = synth::speckle_projector();
    return e;
}

}  // namespace

Session::~Session() {
    stop_scan();
    if (pipeline_) pipeline_->stop();
}

Result<std::unique_ptr<Session>> Session::open(bool force_emulator, UpdateSink updates, ButtonSink buttons) {
    auto s = std::unique_ptr<Session>(new Session());
    std::unique_ptr<usb::Transport> transport;
    std::shared_ptr<EmulatorScene> emu;

    if (!force_emulator) {
        if (auto devices = usb::enumerate_devices(); devices && !devices->empty()) {
            auto t = usb::open_libusb(devices->front());
            if (!t) return std::unexpected(t.error());
            transport = std::move(*t);
        }
    }
    if (!transport) {
        // Emulator: put the real calibration into its flash when EXStar's cache is available, so the
        // app goes through exactly the same calibration path as with hardware.
        auto sim = std::make_unique<sim::SimTransport>();
        RigCalibration rig = synth::synthetic_einstar_rig();
        if (auto blob = calib::encode_quick_flash_blob_from_directory(kExstarCalibrationCache)) {
            sim->set_flash(0, *blob);
            if (auto cal = calib::decode_flash_blob(*blob)) rig = cal->rig();
        }
        emu = make_scene(rig);
        const SE3 T_left_right = rig.T_right_left.inverse();
        const double half_toe = 0.5 * rotation_angle(rig.T_right_left);
        sim->set_frame_provider([emu, T_left_right, half_toe](int sensor, std::uint32_t frame_id, ImageU8& out) {
            const double t = frame_id * 0.068 / 3.0;  // three groups per 68 ms trigger cycle
            const SE3 T_wl = emu->pose(t);
            synth::Projector p = emu->projector;
            p.T_world_projector = T_wl;
            p.T_world_projector.translation() = T_wl * (0.5 * T_left_right.translation());
            p.T_world_projector.linear() = T_wl.linear() * Eigen::AngleAxisd(-half_toe, Vec3::UnitY()).toRotationMatrix();
            synth::RenderParams rp;
            rp.supersample = 1;
            rp.seed = frame_id * 3 + static_cast<std::uint32_t>(sensor);
            if (sensor == 2) {
                out.fill(0);
                return;
            }
            out = synth::render_view(emu->scene, p, sensor == 0 ? emu->rig.left : emu->rig.right,
                                     sensor == 0 ? T_wl : T_wl * T_left_right, rp)
                      .image;
        });
        transport = std::move(sim);
        s->emulated_ = true;
        s->emulator_state_ = emu;
    }

    device::ConnectOptions opts;
    auto dev = device::EinstarDevice::connect(std::move(transport), opts);
    if (!dev) return std::unexpected(dev.error());
    s->device_ = std::move(*dev);
    s->device_->set_button_sink(std::move(buttons));

    // Calibration straight from the scanner's flash (read-only).
    RigCalibration rig;
    auto blob = s->device_->read_flash(0, calib::kFlashBlobSize);
    if (!blob) return std::unexpected(blob.error());
    if (auto cal = calib::decode_flash_blob(*blob)) {
        rig = cal->rig();
        log::info("calibration {} from device flash, baseline {:.3f} mm", cal->calibration_time, rig.baseline_mm());
    } else if (s->emulated_) {
        rig = emu->rig;
    } else {
        return make_error(Errc::protocol, "could not decode the scanner's calibration: " + cal.error().message);
    }

    pipeline::ScanPipelineParams pp;
    pp.block_when_full = s->emulated_;  // the emulator can wait; a real scanner cannot
    pp.tracker.deterministic_relocalisation = s->emulated_;  // live: never wait for the relocalisation worker
    auto frontend = std::make_unique<pipeline::StereoFrontend>(rig);
    // Everything needed to re-rectify raw images and tie the scan to this unit, in every session file.
    session::DeviceRecord record;
    const auto& info = s->device_->info();
    record.vendor = info.vendor_name;
    record.product = info.product_name;
    record.serial = info.serial;
    record.firmware = info.firmware;
    record.calibration_blob = *blob;
    record.rig = rig;
    record.R_rect_left = frontend->rectification().R_left;
    record.R_rect_right = frontend->rectification().R_right;
    record.rectified = frontend->rectification().rectified;
    s->pipeline_ = std::make_unique<pipeline::ScanPipeline>(std::move(frontend), pp, std::move(updates));
    s->pipeline_->set_device_record(std::move(record));
    // Every scan is recorded so the process step can use every frame.
    if (const char* dir = std::getenv("EINSTAR_SCAN_DIR")) s->pipeline_->set_recording_directory(dir);
    else if (const char* home = std::getenv("HOME")) s->pipeline_->set_recording_directory(std::string(home) + "/Documents/Einstar/Scans");
    s->pipeline_->start();

    s->description_ = std::format("{} {} (serial {}, firmware {})", s->emulated_ ? "Emulated" : "Scanner", info.product_name,
                                  info.serial, info.firmware);
    return s;
}

Result<void> Session::apply(const ScanSettings& st) {
    // Sensors 0 and 1 share one exposure register; gain is per sensor.
    if (auto r = device_->set_exposure(0, static_cast<std::uint32_t>(st.exposure)); !r) return r;
    for (int sensor = 0; sensor < 2; ++sensor)
        if (auto r = device_->set_gain(sensor, static_cast<std::uint16_t>(st.gain)); !r) return r;
    if (auto r = device_->set_laser_percent(st.laser_percent); !r) return r;
    if (auto r = device_->set_strobe(0, st.strobe); !r) return r;
    // Recorded with every frame. The device clamps to the sensors' ranges; read the values back.
    session::CaptureSettings cs;
    for (int sensor = 0; sensor < 2; ++sensor) {
        const auto e = device_->exposure(sensor);
        auto g = device_->gain(sensor);
        // A failed low-byte write can leave a wrong gain (see EinstarDevice::set_gain): write it once more.
        if (g && std::abs(static_cast<int>(*g) - st.gain) > 4) {
            log::warn("gain of sensor {} reads {} after writing {}; writing again", sensor, *g, st.gain);
            if (auto r = device_->set_gain(sensor, static_cast<std::uint16_t>(st.gain)); !r) return r;
            g = device_->gain(sensor);
        }
        cs.exposure[static_cast<std::size_t>(sensor)] = e ? *e : static_cast<std::uint32_t>(st.exposure);
        cs.gain[static_cast<std::size_t>(sensor)] = g ? *g : static_cast<std::uint16_t>(st.gain);
    }
    cs.laser_percent = std::clamp(st.laser_percent, 0, 100);
    cs.strobe = std::clamp(st.strobe, 0, device::kMaxStrobeLuminance);
    cs.trigger_period_us = st.trigger_period_us;
    pipeline_->set_capture_settings(cs);
    return {};
}

Result<void> Session::start_scan(const ScanSettings& st) {
    if (scanning_) return {};
    if (auto r = apply(st); !r) return r;
    if (auto r = device_->configure_scan_mode(st.trigger_period_us); !r) return r;
    auto r = device_->start_stream([this](usb::FrameGroup&& g) { pipeline_->push(std::move(g)); });
    if (!r) return r;
    scanning_ = true;
    return {};
}

void Session::stop_scan() {
    if (!scanning_) return;
    (void)device_->set_trigger(0, 0);
    (void)device_->set_laser_percent(0);
    (void)device_->set_strobe(0, 0);
    device_->stop_stream();
    scanning_ = false;
}

void Session::set_distance_indication(float mean_depth_mm) {
    // Three LED zones: near / good / far around the 250-450 mm sweet spot (mapping unverified on hardware).
    const int zone = mean_depth_mm <= 0 ? last_zone_ : mean_depth_mm < 250 ? 0 : mean_depth_mm > 450 ? 2 : 1;
    if (zone < 0 || zone == last_zone_) return;
    last_zone_ = zone;
    (void)device_->set_indication(static_cast<device::DistanceIndication>(zone));
}

}  // namespace einstar::app
