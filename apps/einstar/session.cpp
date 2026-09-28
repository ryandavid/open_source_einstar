#include "session.hpp"

#include <cmath>
#include <filesystem>
#include <format>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/log.hpp"
#include "einstar/sim/sim_transport.hpp"
#include "einstar/synth/speckle_scene.hpp"

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
        const Vec3 fwd = (target - eye).normalized();
        const Vec3 right = -Vec3(0, 1, 0).cross(fwd).normalized();
        const Vec3 down = fwd.cross(right);
        SE3 T = SE3::Identity();
        T.linear().col(0) = right;
        T.linear().col(1) = down;
        T.linear().col(2) = fwd;
        T.translation() = eye;
        const double half_toe = 0.5 * rotation_angle(rig.T_right_left);
        T.linear() = T.linear() * Eigen::AngleAxisd(half_toe, Vec3::UnitY()).toRotationMatrix();
        return T;
    }
};

std::shared_ptr<EmulatorScene> make_scene(const RigCalibration& rig) {
    auto e = std::make_shared<EmulatorScene>();
    e->rig = rig;
    auto box = [&](Vec3 c, Vec3 half, double yaw_deg, double tilt_deg) {
        SE3 T = SE3::Identity();
        T.linear() = (Eigen::AngleAxisd(yaw_deg * M_PI / 180, Vec3::UnitY()) *
                      Eigen::AngleAxisd(tilt_deg * M_PI / 180, Vec3::UnitX())).toRotationMatrix();
        T.translation() = c;
        e->scene.primitives.push_back(synth::Box{T, half});
    };
    e->scene.primitives.push_back(synth::Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    e->scene.primitives.push_back(synth::Sphere{Vec3(-80, 20, 20), 45.0});
    box(Vec3(-10, 45, 40), Vec3(25, 25, 18), 30, 0);
    box(Vec3(-150, 50, -10), Vec3(20, 20, 30), -20, 0);
    box(Vec3(-60, 55, -70), Vec3(35, 15, 12), 55, 0);
    box(Vec3(-120, 30, 70), Vec3(12, 40, 12), 10, 15);
    e->scene.primitives.push_back(synth::Sphere{Vec3(20, 55, -40), 15.0});
    e->projector.model.fx = e->projector.model.fy = 800;
    e->projector.model.cx = 640;
    e->projector.model.cy = 400;
    e->projector.pattern = synth::DotPattern::random(1280, 800, 9000, 3.5, 11);
    return e;
}

// Synthetic rig with the Einstar's geometry when no real calibration is available.
RigCalibration synthetic_rig() {
    RigCalibration rig;
    rig.left.width = rig.right.width = 1280;
    rig.left.height = rig.right.height = 1024;
    rig.left.fx = rig.left.fy = 1157.3;
    rig.left.cx = 625.4;
    rig.left.cy = 522.4;
    rig.left.dist = {-0.156, 0.158, 0, 0.0003, 0.039};
    rig.right = rig.left;
    rig.right.cx = 633.8;
    rig.right.cy = 506.0;
    SE3 T = SE3::Identity();
    T.linear() = Eigen::AngleAxisd(-22.15 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    T.translation() = -T.linear() * Vec3(156.9, 0.2, -30.7);
    rig.T_right_left = T;
    return rig;
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
        RigCalibration rig = synthetic_rig();
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
    s->pipeline_ = std::make_unique<pipeline::ScanPipeline>(std::make_unique<pipeline::StereoFrontend>(rig), pp, std::move(updates));
    s->pipeline_->start();

    const auto& info = s->device_->info();
    s->description_ = std::format("{} {} (serial {}, firmware {})", s->emulated_ ? "Emulated" : "Scanner", info.product_name,
                                  info.serial, info.firmware);
    return s;
}

Result<void> Session::apply(const ScanSettings& st) {
    for (int sensor = 0; sensor < 2; ++sensor) {
        if (auto r = device_->set_exposure(sensor, static_cast<std::uint32_t>(st.exposure)); !r) return r;
        if (auto r = device_->set_gain(sensor, static_cast<std::uint16_t>(st.gain)); !r) return r;
    }
    if (auto r = device_->set_laser_percent(st.laser_percent); !r) return r;
    return device_->set_strobe(0, st.strobe);
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
    (void)device_->set_indication(static_cast<device::DistanceIndication>(zone), true);
}

}  // namespace einstar::app
