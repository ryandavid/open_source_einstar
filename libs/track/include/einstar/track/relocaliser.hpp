#pragma once

// Global relocalisation off the tracking thread. Building the descriptor model (FPFH over the whole
// surface, 0.2-0.4 s on a medium scan and growing with it) and registering a frame against it
// (15-50 ms) run on a worker, so a lost frame costs the tracking thread no more than a tracked one.
//
// Requests are stamped with the tracker's frame number and their results are due a fixed number of
// frames later. In deterministic mode (replays, tests) the tracker waits at the due frame if the
// worker is not finished, so the outcome does not depend on timing; live, results are used whenever
// they are ready and the tracker never waits.

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "einstar/track/global_registration.hpp"
#include "einstar/track/tsdf.hpp"

namespace einstar::track {

class GlobalRelocaliser {
public:
    GlobalRelocaliser(GlobalRegistrationParams params, bool deterministic);
    ~GlobalRelocaliser();
    GlobalRelocaliser(const GlobalRelocaliser&) = delete;
    GlobalRelocaliser& operator=(const GlobalRelocaliser&) = delete;

    // Descriptor model from a surface snapshot (any order: sorted and downsampled on the worker),
    // current from `due_frame`.
    void submit_model(std::vector<SurfacePoint> surface, std::int64_t due_frame);
    [[nodiscard]] bool model_pending() const { return model_job_.valid(); }
    // The model to query with at frame `now` (adopting a finished rebuild); null before the first one.
    // `wait_ms` accumulates time spent waiting for a due rebuild (deterministic mode only).
    [[nodiscard]] std::shared_ptr<const FeatureModel> model(std::int64_t now, double& wait_ms);

    // Registers a frame cloud against `model`; the result is due at `due_frame`.
    void submit_query(std::shared_ptr<const FeatureModel> model, OrientedCloud frame_cloud, std::uint32_t seed, std::int64_t due_frame);
    [[nodiscard]] bool query_pending() const { return query_job_.valid(); }
    // The query's result once it is due (and, live, ready): the outer optional is empty while there is
    // nothing to take, the inner one when registration found no match.
    [[nodiscard]] std::optional<std::optional<GlobalRegistrationResult>> take_result(std::int64_t now, double& wait_ms);

    // Drops the model and any outstanding work (waits for a running job).
    void clear();

private:
    void post(std::function<void()> job, bool urgent);
    void run(std::stop_token st);

    GlobalRegistrationParams params_;
    bool deterministic_;
    std::shared_ptr<const FeatureModel> model_;
    std::shared_future<std::shared_ptr<const FeatureModel>> model_job_;
    std::int64_t model_due_ = 0;
    std::future<std::optional<GlobalRegistrationResult>> query_job_;
    std::int64_t query_due_ = 0;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::function<void()>> jobs_;
    bool running_job_ = false;
    std::condition_variable idle_cv_;
    std::jthread worker_;
};

}  // namespace einstar::track
