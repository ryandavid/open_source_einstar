#include "einstar/pipeline/replay.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>
#include <future>
#include <map>
#include <unistd.h>

#include "einstar/calib/convention.hpp"
#include "einstar/core/log.hpp"
#include "einstar/usb/constants.hpp"

namespace einstar::pipeline {

Result<ReplayResult> replay_recording(const session::SessionReader& in, const std::string& out_path, const ReplayOptions& options) {
    if (in.raw_count() == 0) return make_error(Errc::invalid_argument, "the recording has no raw IR images (record with \"Keep raw IR images\" on)");
    if (!in.device()) return make_error(Errc::invalid_argument, "the recording has no scanner record (calibration): its raw images cannot be rectified");

    // A recording in the old camera convention (calib/convention.hpp) is replayed in EXStar's: the rig
    // converted, both raw images turned (the old one turned sensor 1 upright, ours turns sensor 0).
    const bool old_convention = in.device()->camera_convention == 0;
    const RigCalibration rig = old_convention ? calib::swap_camera_convention(in.device()->rig) : in.device()->rig;
    auto frontend = std::make_unique<StereoFrontend>(rig, options.frontend);
    session::DeviceRecord device = *in.device();
    device.rig = rig;
    device.camera_convention = 1;
    device.R_rect_left = frontend->rectification().R_left;
    device.R_rect_right = frontend->rectification().R_right;
    device.rectified = frontend->rectification().rectified;
    device.rectified_right = frontend->rectification().rectified_right;

    ScanPipelineParams pp = options.pipeline;
    pp.block_when_full = true;  // every frame, none dropped
    pp.tracker.deterministic_relocalisation = true;
    pp.record_raw_ir = false;
    ReplayResult res;
    std::size_t processed = 0, accepted = 0;  // (worker thread; read after stop())
    ScanPipeline pipe(std::move(frontend), pp, [&](LiveUpdate&& u) {
        ++processed;
        accepted += u.accepted;
    });
    pipe.set_device_record(device);
    const auto out = std::filesystem::absolute(out_path);
    const auto scratch = out.parent_path() / std::format(".replay-{}-{}", ::getpid(), out.stem().string());
    std::error_code ec;
    std::filesystem::remove_all(scratch, ec);
    std::filesystem::create_directories(scratch, ec);
    if (ec) return make_error(Errc::io, std::format("cannot create {}: {}", scratch.string(), ec.message()));
    pipe.set_recording_directory(scratch.string());

    std::map<std::uint64_t, std::size_t> by_index;  // frame number -> recorded frame
    bool capture = false;
    for (std::size_t i = 0; i < in.frame_count(); ++i) {
        by_index[in.meta(i).index] = i;
        capture |= (in.meta(i).flags & session::frame_global_marker_capture) != 0;
    }
    const bool recorded_map = options.recorded_map || !capture;
    if (recorded_map && !in.global_markers().empty()) pipe.set_global_markers(in.global_markers());
    if (old_convention) {
        pipe.set_left_sensor(usb::kLeftSensor);
    } else {
        for (std::size_t i = 0; i < in.frame_count(); ++i)
            if (const auto& ex = in.meta(i).extras; ex && ex->left_sensor >= 0) {
                pipe.set_left_sensor(ex->left_sensor);
                break;
            }
    }
    pipe.start();

    auto phase = ScanPhase::surface;
    bool marker_frames = false;  // the raw frame's recorded counterpart (or the last one before it) was a capture frame
    for (std::size_t i = options.start; i < in.raw_count() && res.pushed + res.skipped < options.count; ++i) {
        auto raw = in.read_raw(i);
        if (!raw) {
            log::warn("replay: raw frame {}: {}", i, raw.error().message);
            continue;
        }
        // Frames the live pipeline dropped have no recorded frame: they belong to the phase around them.
        if (const auto it = by_index.find(raw->index); it != by_index.end()) {
            const auto& meta = in.meta(it->second);
            marker_frames = (meta.flags & session::frame_global_marker_capture) != 0;
            if (meta.extras) pipe.set_capture_settings(meta.extras->capture);
        }
        if (marker_frames && recorded_map) {
            ++res.skipped;
            continue;
        }
        const auto want = marker_frames ? ScanPhase::global_markers : ScanPhase::surface;
        if (want != phase) {
            if (phase == ScanPhase::global_markers) {
                // The surface scan begins: bundle-adjust the capture, as the user's "optimise" does live.
                pipe.drain();
                std::promise<GlobalMarkerReport> done;
                auto report = done.get_future();
                pipe.optimize_global_markers([&done](const GlobalMarkerReport& r) { done.set_value(r); });
                res.global_markers = report.get();
            }
            pipe.set_phase(want);
            phase = want;
        }
        usb::FrameGroup g;
        g.frame_id = static_cast<std::uint32_t>(raw->index);
        g.timestamp = static_cast<std::uint64_t>(std::llround(raw->timestamp_s * 1e6));
        for (auto& [sensor, image] : raw->images) {
            if (sensor < 0 || sensor > 2) continue;
            usb::StreamFrame sf;
            sf.sensor = sensor;
            sf.frame_id = g.frame_id;
            sf.timestamp = g.timestamp;
            sf.pixels = std::move(image);
            if (old_convention) std::ranges::reverse(sf.pixels.pixels());
            g.sensors[static_cast<std::size_t>(sensor)] = std::move(sf);
        }
        pipe.push(std::move(g));
        ++res.pushed;
        if (options.progress) options.progress(res.pushed);
    }
    pipe.drain();
    const std::string written = pipe.flush_recording();
    pipe.stop();
    res.processed = processed;
    res.accepted = accepted;

    std::vector<std::filesystem::path> files;
    for (const auto& e : std::filesystem::directory_iterator(scratch, ec))
        if (e.path().extension() == ".estr") files.push_back(e.path());
    std::ranges::sort(files);
    for (std::size_t k = 0; k < files.size(); ++k) {
        const bool last = written.empty() ? k + 1 == files.size() : files[k] == std::filesystem::path(written);
        const auto dest = last ? out : out.parent_path() / std::format("{}-part{}.estr", out.stem().string(), k + 1);
        std::filesystem::rename(files[k], dest, ec);
        if (ec) return make_error(Errc::io, std::format("cannot move {} to {}: {}", files[k].string(), dest.string(), ec.message()));
        res.files.push_back(dest.string());
    }
    std::ranges::stable_partition(res.files, [&](const std::string& f) { return std::filesystem::path(f) != out; });  // requested one last
    std::filesystem::remove_all(scratch, ec);
    if (res.files.empty()) return make_error(Errc::io, "nothing was recorded");
    return res;
}

}  // namespace einstar::pipeline
