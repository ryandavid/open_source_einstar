#pragma once

// Metal-backed DeviceFrameData / DeviceRaycastData. On unified memory the buffers are shared and read
// in place. On a discrete GPU they are managed (see gpu::Context::mirrored_buffer): the GPU copy is
// authoritative and the first CPU read copies it back (pass the context to enable that).

#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "einstar/core/device_data.hpp"
#include "einstar/gpu/context.hpp"

namespace einstar::gpu {

class MetalFrameData final : public DeviceFrameData {
public:
    MetalFrameData(int w, int h, Ref<MTL::Buffer> points, Ref<MTL::Buffer> normals, Ref<MTL::Buffer> weights,
                   std::function<void()> on_release = {}, std::shared_ptr<Context> ctx = {})
        : w_(w), h_(h), points_(std::move(points)), normals_(std::move(normals)), weights_(std::move(weights)),
          on_release_(std::move(on_release)), ctx_(std::move(ctx)) {}
    ~MetalFrameData() override {
        if (on_release_) on_release_();
    }

    [[nodiscard]] int width() const override { return w_; }
    [[nodiscard]] int height() const override { return h_; }
    [[nodiscard]] const float* points_xyzw() const override { return static_cast<const float*>(cpu(points_)); }
    [[nodiscard]] const float* normals_xyzw() const override { return static_cast<const float*>(cpu(normals_)); }
    [[nodiscard]] const float* weights() const override { return static_cast<const float*>(cpu(weights_)); }

    [[nodiscard]] MTL::Buffer* points_buffer() const { return points_.get(); }
    [[nodiscard]] MTL::Buffer* normals_buffer() const { return normals_.get(); }
    [[nodiscard]] MTL::Buffer* weights_buffer() const { return weights_.get(); }
    // The producer already copied the GPU results back (gpu::Context::sync_for_cpu): no lazy sync.
    void mark_cpu_current() const { std::call_once(synced_, [] {}); }

private:
    // CPU contents, brought up to date with the GPU's writes on first access.
    void* cpu(const Ref<MTL::Buffer>& b) const {
        if (ctx_) std::call_once(synced_, [this] { ctx_->sync_now({points_.get(), normals_.get(), weights_.get()}); });
        return b->contents();
    }

    int w_, h_;
    Ref<MTL::Buffer> points_, normals_, weights_;
    std::function<void()> on_release_;
    std::shared_ptr<Context> ctx_;
    mutable std::once_flag synced_;
};

class MetalRaycastData final : public DeviceRaycastData {
public:
    MetalRaycastData(int w, int h, Ref<MTL::Buffer> points, Ref<MTL::Buffer> normals, std::shared_ptr<Context> ctx = {})
        : w_(w), h_(h), points_(std::move(points)), normals_(std::move(normals)), ctx_(std::move(ctx)) {}
    [[nodiscard]] int width() const override { return w_; }
    [[nodiscard]] int height() const override { return h_; }
    [[nodiscard]] const float* points_xyzw() const override { return static_cast<const float*>(cpu(points_)); }
    [[nodiscard]] const float* normals_xyzw() const override { return static_cast<const float*>(cpu(normals_)); }
    [[nodiscard]] MTL::Buffer* points_buffer() const { return points_.get(); }
    [[nodiscard]] MTL::Buffer* normals_buffer() const { return normals_.get(); }

private:
    void* cpu(const Ref<MTL::Buffer>& b) const {
        if (ctx_) std::call_once(synced_, [this] { ctx_->sync_now({points_.get(), normals_.get()}); });
        return b->contents();
    }

    int w_, h_;
    Ref<MTL::Buffer> points_, normals_;
    std::shared_ptr<Context> ctx_;
    mutable std::once_flag synced_;
};

}  // namespace einstar::gpu
