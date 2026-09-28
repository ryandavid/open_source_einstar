#include "einstar/gpu/profile.hpp"

#include <chrono>
#include <cstdlib>
#include <format>
#include <map>
#include <mutex>
#include <vector>

namespace einstar::gpu::profile {
namespace {

struct Entry {
    int calls = 0;
    double gpu_ms = 0, wall_ms = 0;
};

std::mutex& mutex() {
    static std::mutex m;
    return m;
}
std::map<std::string, Entry>& entries() {
    static std::map<std::string, Entry> e;
    return e;
}

double gpu_ms_of(MTL::CommandBuffer* cmd) { return (cmd->GPUEndTime() - cmd->GPUStartTime()) * 1000.0; }

}  // namespace

bool enabled() {
    static const bool on = [] {
        const char* v = std::getenv("EINSTAR_GPU_PROFILE");
        return v && *v && *v != '0';
    }();
    return on;
}

void record(std::string_view label, MTL::CommandBuffer* cmd, double wall_ms) {
    if (!enabled() || !cmd) return;
    std::lock_guard lock(mutex());
    auto& e = entries()[std::string(label)];
    ++e.calls;
    e.gpu_ms += gpu_ms_of(cmd);
    e.wall_ms += wall_ms;
}

void commit_and_wait(MTL::CommandBuffer* cmd, std::string_view label) {
    const auto t0 = std::chrono::steady_clock::now();
    cmd->commit();
    cmd->waitUntilCompleted();
    if (enabled()) record(label, cmd, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
}

void split(MTL::CommandQueue* queue, MTL::CommandBuffer*& cmd, MTL::ComputeCommandEncoder*& enc, std::string_view label) {
    if (!enabled()) return;
    enc->endEncoding();
    commit_and_wait(cmd, label);
    cmd = queue->commandBuffer();
    enc = cmd->computeCommandEncoder();
}

std::string report(int frames) {
    std::lock_guard lock(mutex());
    std::vector<std::pair<std::string, Entry>> rows(entries().begin(), entries().end());
    std::string out = std::format("{:34} {:>7} {:>10} {:>10} {:>12}\n", "stage", "calls", "gpu ms", "wall ms", "gpu ms/frame");
    double total = 0;
    for (const auto& [label, e] : rows) {
        const double per_frame = frames > 0 ? e.gpu_ms / frames : 0.0;
        total += per_frame;
        out += std::format("{:34} {:7} {:10.3f} {:10.3f} {:12.3f}\n", label, e.calls, e.gpu_ms / e.calls, e.wall_ms / e.calls, per_frame);
    }
    if (frames > 0) out += std::format("{:34} {:7} {:>10} {:>10} {:12.3f}\n", "total", "", "", "", total);
    return out;
}

void reset() {
    std::lock_guard lock(mutex());
    entries().clear();
}

}  // namespace einstar::gpu::profile
