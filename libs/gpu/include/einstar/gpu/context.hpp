#pragma once

// Thin RAII layer over metal-cpp. Shaders are compiled at runtime from source so the build
// does not need the offline `metal` compiler (only the Command Line Tools are required).

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include <Metal/Metal.hpp>

#include "einstar/core/error.hpp"

namespace einstar::gpu {

// Owning wrapper for NS::Object-derived handles (released on destruction).
template <typename T>
class Ref {
public:
    Ref() = default;
    explicit Ref(T* p) : ptr_(p) {}  // adopts a +1 reference
    Ref(const Ref& o) : ptr_(o.ptr_) { if (ptr_) ptr_->retain(); }
    Ref(Ref&& o) noexcept : ptr_(std::exchange(o.ptr_, nullptr)) {}
    Ref& operator=(Ref o) noexcept { std::swap(ptr_, o.ptr_); return *this; }
    ~Ref() { if (ptr_) ptr_->release(); }

    static Ref retain(T* p) { if (p) p->retain(); return Ref(p); }

    [[nodiscard]] T* get() const { return ptr_; }
    [[nodiscard]] T* operator->() const { return ptr_; }
    [[nodiscard]] explicit operator bool() const { return ptr_ != nullptr; }

private:
    T* ptr_ = nullptr;
};

class Context {
public:
    // Uses the system default device, or adopts an existing one (e.g. the app's device).
    static Result<std::shared_ptr<Context>> create(MTL::Device* device = nullptr);

    [[nodiscard]] MTL::Device* device() const { return device_.get(); }
    [[nodiscard]] MTL::CommandQueue* queue() const { return queue_.get(); }

    // Compiles (and caches by name) a library from Metal source.
    Result<MTL::Library*> library(std::string_view name, std::string_view source);
    Result<Ref<MTL::ComputePipelineState>> compute_pipeline(MTL::Library* lib, std::string_view function);

    [[nodiscard]] Ref<MTL::Buffer> buffer(std::size_t bytes, MTL::ResourceOptions options = MTL::ResourceStorageModeShared);

private:
    Ref<MTL::Device> device_;
    Ref<MTL::CommandQueue> queue_;
    std::unordered_map<std::string, Ref<MTL::Library>> libraries_;
};

[[nodiscard]] NS::String* ns_string(std::string_view s);  // autoreleased

}  // namespace einstar::gpu
