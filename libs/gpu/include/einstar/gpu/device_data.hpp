#pragma once

// Metal-backed DeviceFrameData / DeviceRaycastData (shared storage: CPU-readable in place).

#include <functional>
#include <utility>

#include "einstar/core/device_data.hpp"
#include "einstar/gpu/context.hpp"

namespace einstar::gpu {

class MetalFrameData final : public DeviceFrameData {
public:
    MetalFrameData(int w, int h, Ref<MTL::Buffer> points, Ref<MTL::Buffer> normals, Ref<MTL::Buffer> weights,
                   std::function<void()> on_release = {})
        : w_(w), h_(h), points_(std::move(points)), normals_(std::move(normals)), weights_(std::move(weights)),
          on_release_(std::move(on_release)) {}
    ~MetalFrameData() override {
        if (on_release_) on_release_();
    }

    [[nodiscard]] int width() const override { return w_; }
    [[nodiscard]] int height() const override { return h_; }
    [[nodiscard]] const float* points_xyzw() const override { return static_cast<const float*>(points_->contents()); }
    [[nodiscard]] const float* normals_xyzw() const override { return static_cast<const float*>(normals_->contents()); }
    [[nodiscard]] const float* weights() const override { return static_cast<const float*>(weights_->contents()); }

    [[nodiscard]] MTL::Buffer* points_buffer() const { return points_.get(); }
    [[nodiscard]] MTL::Buffer* normals_buffer() const { return normals_.get(); }
    [[nodiscard]] MTL::Buffer* weights_buffer() const { return weights_.get(); }

private:
    int w_, h_;
    Ref<MTL::Buffer> points_, normals_, weights_;
    std::function<void()> on_release_;
};

class MetalRaycastData final : public DeviceRaycastData {
public:
    MetalRaycastData(int w, int h, Ref<MTL::Buffer> points, Ref<MTL::Buffer> normals)
        : w_(w), h_(h), points_(std::move(points)), normals_(std::move(normals)) {}
    [[nodiscard]] int width() const override { return w_; }
    [[nodiscard]] int height() const override { return h_; }
    [[nodiscard]] const float* points_xyzw() const override { return static_cast<const float*>(points_->contents()); }
    [[nodiscard]] const float* normals_xyzw() const override { return static_cast<const float*>(normals_->contents()); }
    [[nodiscard]] MTL::Buffer* points_buffer() const { return points_.get(); }
    [[nodiscard]] MTL::Buffer* normals_buffer() const { return normals_.get(); }

private:
    int w_, h_;
    Ref<MTL::Buffer> points_, normals_;
};

}  // namespace einstar::gpu
