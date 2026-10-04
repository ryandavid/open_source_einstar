// einstar-cli replay: a recording's raw IR images through the live pipeline again (stereo, markers,
// tracking, fusion) as this build does it, into a new recording (pipeline::replay_recording). Replay a
// scan before and after a change to the depth front end, then `process` / `inspect` both results.

#include <cstdlib>
#include <print>
#include <span>
#include <string_view>

#include "einstar/pipeline/replay.hpp"

namespace einstar::cli {
namespace {

const char* option(std::span<char*> args, std::string_view name) {
    for (std::size_t i = 0; i + 1 < args.size(); ++i)
        if (name == args[i]) return args[i + 1];
    return nullptr;
}

bool flag(std::span<char*> args, std::string_view name) {
    for (const char* a : args)
        if (name == a) return true;
    return false;
}

}  // namespace

int replay_cmd(const char* path, std::span<char*> args) {
    const char* out = option(args, "-o");
    if (!out) {
        std::println(stderr, "replay needs -o out.estr");
        return 2;
    }
    auto in = session::SessionReader::open(path);
    if (!in) {
        std::println(stderr, "{}", in.error().message);
        return 1;
    }
    pipeline::ReplayOptions o;
    if (flag(args, "--cpu")) o.frontend.backend = pipeline::StereoBackend::cpu;
    if (const char* v = option(args, "--reference-depth")) o.frontend.reference_depth_mm = std::atof(v);
    o.recorded_map = flag(args, "--recorded-map");
    if (const char* v = option(args, "--start")) o.start = static_cast<std::size_t>(std::max(0L, std::atol(v)));
    if (const char* v = option(args, "--count")) o.count = static_cast<std::size_t>(std::max(0L, std::atol(v)));
    o.progress = [](std::size_t n) {
        if (n % 200 == 0) std::println("{} frames", n);
    };
    auto r = pipeline::replay_recording(**in, out, o);
    if (!r) {
        std::println(stderr, "{}", r.error().message);
        return 1;
    }
    if (r->global_markers) {
        if (!r->global_markers->error.empty()) std::println("global markers: {}", r->global_markers->error);
        else std::println("global markers: {} markers from {} keyframes", r->global_markers->markers, r->global_markers->keyframes);
    }
    for (std::size_t k = 0; k + 1 < r->files.size(); ++k) std::println("also wrote {} (tracking restarted)", r->files[k]);
    std::println("replayed {} raw frames{}: {} processed, {} accepted -> {}", r->pushed,
                 r->skipped ? std::format(" ({} marker-capture frames skipped)", r->skipped) : std::string(), r->processed, r->accepted,
                 r->files.back());
    return 0;
}

}  // namespace einstar::cli
