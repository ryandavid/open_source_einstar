#pragma once

// Replaying a recording's raw IR images through the live pipeline (stereo, markers, tracking, fusion)
// as this build does it, into a new recording: one scan recorded with raw IR becomes a fixed test case
// for every later change to the depth front end. Frames are processed in order and none is dropped.

#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "einstar/core/error.hpp"
#include "einstar/pipeline/scan_pipeline.hpp"
#include "einstar/session/session.hpp"

namespace einstar::pipeline {

struct ReplayOptions {
    StereoFrontendParams frontend;
    ScanPipelineParams pipeline;  // block_when_full and deterministic relocalisation are forced on
    // Use the recording's global-marker map and replay only the surface frames; otherwise a recorded
    // marker capture is replayed as one and bundle-adjusted where the surface scan begins.
    bool recorded_map = false;
    std::size_t start = 0;  // first raw frame
    std::size_t count = std::numeric_limits<std::size_t>::max();
    std::function<void(std::size_t pushed)> progress;
};

struct ReplayResult {
    // The recordings written: the last is the requested path; a tracking restart without a fixed marker map
    // starts a new file, written beside it as <name>-part<k>.estr.
    std::vector<std::string> files;
    std::size_t pushed = 0;   // raw frames replayed
    std::size_t skipped = 0;  // marker-capture frames left out (recorded_map)
    std::size_t processed = 0, accepted = 0;
    std::optional<GlobalMarkerReport> global_markers;
};

[[nodiscard]] Result<ReplayResult> replay_recording(const session::SessionReader& in, const std::string& out_path,
                                                    const ReplayOptions& options = {});

}  // namespace einstar::pipeline
