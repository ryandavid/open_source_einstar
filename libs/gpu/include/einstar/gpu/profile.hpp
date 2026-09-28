#pragma once

// GPU profiling (EINSTAR_GPU_PROFILE=1). Every command buffer the modules submit is recorded under a
// label with its GPU execution time and the CPU wall time from commit to completion. split() cuts a
// command buffer at a stage boundary so each stage is timed on its own. That serialises the GPU
// work, so profiling shows where time goes, not the optimised total. When profiling is off,
// split() does nothing and record() only returns.

#include <string>
#include <string_view>

#include <Metal/Metal.hpp>

namespace einstar::gpu::profile {

[[nodiscard]] bool enabled();

// `cmd` must have completed; `wall_ms` is the CPU time from commit to completion (0 if unknown).
void record(std::string_view label, MTL::CommandBuffer* cmd, double wall_ms = 0.0);

// Commits `cmd` and waits; records it when profiling.
void commit_and_wait(MTL::CommandBuffer* cmd, std::string_view label);

// Profiling only: ends `enc`, commits and waits for `cmd` (recorded under `label`), then continues
// in a new command buffer and compute encoder from `queue`.
void split(MTL::CommandQueue* queue, MTL::CommandBuffer*& cmd, MTL::ComputeCommandEncoder*& enc, std::string_view label);

// Table of labels: calls, GPU ms per call, wall ms per call, and per frame when `frames` > 0.
[[nodiscard]] std::string report(int frames = 0);
void reset();

}  // namespace einstar::gpu::profile
