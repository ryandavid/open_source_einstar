#include "einstar/track/relocaliser.hpp"

#include <tbb/task_arena.h>

#include "einstar/core/timing.hpp"

namespace einstar::track {

GlobalRelocaliser::GlobalRelocaliser(GlobalRegistrationParams params, bool deterministic)
    : params_(params), deterministic_(deterministic), worker_([this](std::stop_token st) { run(st); }) {}

GlobalRelocaliser::~GlobalRelocaliser() {
    clear();
    worker_.request_stop();
    cv_.notify_all();
}

void GlobalRelocaliser::post(std::function<void()> job, bool urgent) {
    {
        std::lock_guard lock(mutex_);
        if (urgent) jobs_.push_front(std::move(job));
        else jobs_.push_back(std::move(job));
    }
    cv_.notify_one();
}

void GlobalRelocaliser::run(std::stop_token st) {
    // Two threads for the parallel parts: enough to finish well inside the due frames without
    // competing with the live pipeline for every core.
    tbb::task_arena arena(2);
    while (true) {
        std::function<void()> job;
        {
            std::unique_lock lock(mutex_);
            cv_.wait(lock, [&] { return st.stop_requested() || !jobs_.empty(); });
            if (jobs_.empty()) return;  // stop requested and nothing left
            job = std::move(jobs_.front());
            jobs_.pop_front();
            running_job_ = true;
        }
        arena.execute(job);
        {
            std::lock_guard lock(mutex_);
            running_job_ = false;
        }
        idle_cv_.notify_all();
    }
}

void GlobalRelocaliser::submit_model(std::vector<SurfacePoint> surface, std::int64_t due_frame) {
    auto promise = std::make_shared<std::promise<std::shared_ptr<const FeatureModel>>>();
    model_job_ = promise->get_future().share();
    model_due_ = due_frame;
    post(
        [promise, sorted = std::move(surface), p = params_]() mutable {
            sort_canonical(sorted);
            OrientedCloud cloud;
            cloud.points.reserve(sorted.size());
            cloud.normals.reserve(sorted.size());
            for (const auto& sp : sorted) {
                cloud.points.push_back(sp.position);
                cloud.normals.push_back(sp.normal);
            }
            promise->set_value(std::make_shared<const FeatureModel>(voxel_downsample(cloud, p.voxel_mm), p.feature_radius_mm));
        },
        false);
}

std::shared_ptr<const FeatureModel> GlobalRelocaliser::model(std::int64_t now, double& wait_ms) {
    if (model_job_.valid()) {
        const bool due = now >= model_due_;
        if (deterministic_ && due && model_job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            Stopwatch sw;
            model_job_.wait();
            wait_ms += sw.elapsed_ms();
        }
        // Live, a finished rebuild is adopted at once; deterministic runs adopt it exactly when due.
        const bool ready = model_job_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
        if (ready && (due || !deterministic_)) {
            model_ = model_job_.get();
            model_job_ = {};
        }
    }
    return model_;
}

void GlobalRelocaliser::submit_query(std::shared_ptr<const FeatureModel> model, OrientedCloud frame_cloud, std::uint32_t seed,
                                     std::int64_t due_frame) {
    auto promise = std::make_shared<std::promise<std::optional<GlobalRegistrationResult>>>();
    query_job_ = promise->get_future();
    query_due_ = due_frame;
    post(
        [promise, feature_model = std::move(model), cloud = std::move(frame_cloud), p = params_, seed] {
            promise->set_value(register_global(cloud, *feature_model, p, seed));
        },
        true);  // ahead of a model rebuild: a lost scanner is waiting on it
}

std::optional<std::optional<GlobalRegistrationResult>> GlobalRelocaliser::take_result(std::int64_t now, double& wait_ms) {
    if (!query_job_.valid()) return std::nullopt;
    if (deterministic_) {
        if (now < query_due_) return std::nullopt;
        Stopwatch sw;
        query_job_.wait();
        wait_ms += sw.elapsed_ms();
    } else if (query_job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return std::nullopt;
    }
    return query_job_.get();
}

void GlobalRelocaliser::clear() {
    {
        std::unique_lock lock(mutex_);
        jobs_.clear();
        idle_cv_.wait(lock, [&] { return !running_job_; });
    }
    // Queued jobs were dropped without setting their promises: forget the futures.
    model_job_ = {};
    query_job_ = {};
    model_.reset();
}

}  // namespace einstar::track
