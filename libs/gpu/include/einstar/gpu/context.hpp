#pragma once

// Thin RAII layer over metal-cpp. Shaders are compiled at runtime from source so the build
// does not need the offline `metal` compiler (only the Command Line Tools are required).

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <string>
#include <span>
#include <string_view>
#include <unordered_map>
#include <utility>

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
    // Specialises `uint` function constants ([[function_constant(index)]]) at pipeline creation.
    Result<Ref<MTL::ComputePipelineState>> compute_pipeline(MTL::Library* lib, std::string_view function,
                                                            std::span<const std::pair<int, std::uint32_t>> uint_constants);

    [[nodiscard]] Ref<MTL::Buffer> buffer(std::size_t bytes, MTL::ResourceOptions options = MTL::ResourceStorageModeShared);

    // Storage by access pattern. On unified memory (Apple silicon, integrated GPUs) all of these are
    // plain shared buffers and the sync helpers do nothing. On a discrete GPU, shared buffers live in
    // system memory and every GPU access crosses PCIe, so:
    //  - gpu_buffer: working memory the CPU never touches -> private (VRAM);
    //  - mirrored_buffer: memory both sides use -> managed (a VRAM copy plus a CPU copy): after CPU
    //    writes call cpu_modified(), before CPU reads of GPU results encode sync_for_cpu().
    [[nodiscard]] bool unified_memory() const { return unified_; }
    // Apple-family GPU (Apple silicon). Kernels tuned for other GPUs are selected when this is false,
    // so the Apple paths stay exactly as they are.
    [[nodiscard]] bool apple_gpu() const { return apple_gpu_; }
    [[nodiscard]] Ref<MTL::Buffer> gpu_buffer(std::size_t bytes);
    [[nodiscard]] Ref<MTL::Buffer> mirrored_buffer(std::size_t bytes);
    // Publishes CPU writes to a managed buffer (no-op for other storage modes).
    static void cpu_modified(MTL::Buffer* buffer, std::size_t offset, std::size_t bytes);
    static void cpu_modified(MTL::Buffer* buffer);
    // Encodes the copy-back of GPU writes to managed buffers so the CPU sees them once `cmd` completes
    // (nothing is encoded for other storage modes).
    static void sync_for_cpu(MTL::CommandBuffer* cmd, std::initializer_list<MTL::Buffer*> buffers);
    // The same, run now and waited for (no GPU work when none of the buffers is managed).
    void sync_now(std::initializer_list<MTL::Buffer*> buffers);
    // Synchronous CPU access to a buffer of any storage mode (private ones go through a staging copy).
    void fill(MTL::Buffer* buffer, std::uint8_t value);
    void upload(MTL::Buffer* buffer, std::size_t offset, const void* src, std::size_t bytes);
    void download(MTL::Buffer* buffer, std::size_t offset, void* dst, std::size_t bytes);

private:
    bool unified_ = true;
    bool apple_gpu_ = true;
    Ref<MTL::Device> device_;
    Ref<MTL::CommandQueue> queue_;
    std::unordered_map<std::string, Ref<MTL::Library>> libraries_;
};

[[nodiscard]] NS::String* ns_string(std::string_view s);  // autoreleased

}  // namespace einstar::gpu
