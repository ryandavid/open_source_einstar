#include "einstar/gpu/context.hpp"

#include <algorithm>
#include <cstring>
#include <format>

#include <Foundation/Foundation.hpp>

namespace einstar::gpu {

NS::String* ns_string(std::string_view s) {
    return NS::String::string(std::string(s).c_str(), NS::UTF8StringEncoding);
}

Result<std::shared_ptr<Context>> Context::create(MTL::Device* device) {
    auto ctx = std::make_shared<Context>();
    ctx->device_ = device ? Ref<MTL::Device>::retain(device) : Ref<MTL::Device>(MTL::CreateSystemDefaultDevice());
    if (!ctx->device_) return make_error(Errc::unsupported, "no Metal device");
    ctx->queue_ = Ref<MTL::CommandQueue>(ctx->device_->newCommandQueue());
    ctx->unified_ = ctx->device_->hasUnifiedMemory();
    ctx->apple_gpu_ = ctx->device_->supportsFamily(MTL::GPUFamilyApple1);
    return ctx;
}

Result<MTL::Library*> Context::library(std::string_view name, std::string_view source) {
    if (auto it = libraries_.find(std::string(name)); it != libraries_.end()) return it->second.get();
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    NS::Error* err = nullptr;
    auto* options = MTL::CompileOptions::alloc()->init();
    options->setFastMathEnabled(true);
    MTL::Library* lib = device_->newLibrary(ns_string(source), options, &err);
    options->release();
    if (!lib) {
        std::string msg = err ? err->localizedDescription()->utf8String() : "unknown error";
        pool->release();
        return make_error(Errc::invalid_argument, std::format("Metal compile of '{}' failed: {}", name, msg));
    }
    pool->release();
    auto& slot = libraries_[std::string(name)];
    slot = Ref<MTL::Library>(lib);
    return slot.get();
}

Result<Ref<MTL::ComputePipelineState>> Context::compute_pipeline(MTL::Library* lib, std::string_view function) {
    return compute_pipeline(lib, function, {});
}

Result<Ref<MTL::ComputePipelineState>> Context::compute_pipeline(MTL::Library* lib, std::string_view function,
                                                                 std::span<const std::pair<int, std::uint32_t>> uint_constants) {
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    NS::Error* err = nullptr;
    Ref<MTL::Function> fn;
    if (uint_constants.empty()) {
        fn = Ref<MTL::Function>(lib->newFunction(ns_string(function)));
    } else {
        Ref<MTL::FunctionConstantValues> values(MTL::FunctionConstantValues::alloc()->init());
        for (const auto& [index, value] : uint_constants)
            values->setConstantValue(&value, MTL::DataTypeUInt, static_cast<NS::UInteger>(index));
        fn = Ref<MTL::Function>(lib->newFunction(ns_string(function), values.get(), &err));
    }
    if (!fn) {
        std::string msg = err ? err->localizedDescription()->utf8String() : "not found";
        pool->release();
        return make_error(Errc::not_found, std::format("Metal function '{}': {}", function, msg));
    }
    Ref<MTL::ComputePipelineState> pso(device_->newComputePipelineState(fn.get(), &err));
    if (!pso) {
        std::string msg = err ? err->localizedDescription()->utf8String() : "unknown error";
        pool->release();
        return make_error(Errc::invalid_argument, std::format("pipeline '{}' failed: {}", function, msg));
    }
    pool->release();
    return pso;
}

Ref<MTL::Buffer> Context::buffer(std::size_t bytes, MTL::ResourceOptions options) {
    return Ref<MTL::Buffer>(device_->newBuffer(std::max<std::size_t>(bytes, 16), options));
}

Ref<MTL::Buffer> Context::gpu_buffer(std::size_t bytes) {
    if (unified_) return buffer(bytes);
    // Private memory is not zeroed by Metal on every driver; callers relied on zeroed shared buffers.
    auto b = buffer(bytes, MTL::ResourceStorageModePrivate);
    fill(b.get(), 0);
    return b;
}

Ref<MTL::Buffer> Context::mirrored_buffer(std::size_t bytes) {
    return buffer(bytes, unified_ ? MTL::ResourceStorageModeShared : MTL::ResourceStorageModeManaged);
}

void Context::cpu_modified(MTL::Buffer* buffer, std::size_t offset, std::size_t bytes) {
    if (buffer && bytes > 0 && buffer->storageMode() == MTL::StorageModeManaged)
        buffer->didModifyRange(NS::Range::Make(offset, bytes));
}

void Context::cpu_modified(MTL::Buffer* buffer) {
    if (buffer) cpu_modified(buffer, 0, buffer->length());
}

void Context::sync_for_cpu(MTL::CommandBuffer* cmd, std::initializer_list<MTL::Buffer*> buffers) {
    MTL::BlitCommandEncoder* be = nullptr;
    for (auto* b : buffers) {
        if (!b || b->storageMode() != MTL::StorageModeManaged) continue;
        if (!be) be = cmd->blitCommandEncoder();
        be->synchronizeResource(b);
    }
    if (be) be->endEncoding();
}

void Context::sync_now(std::initializer_list<MTL::Buffer*> buffers) {
    if (std::ranges::none_of(buffers, [](MTL::Buffer* b) { return b && b->storageMode() == MTL::StorageModeManaged; })) return;
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    MTL::CommandBuffer* cmd = queue_->commandBuffer();
    sync_for_cpu(cmd, buffers);
    cmd->commit();
    cmd->waitUntilCompleted();
    pool->release();
}

void Context::fill(MTL::Buffer* buffer, std::uint8_t value) {
    if (buffer->storageMode() != MTL::StorageModePrivate) {
        std::memset(buffer->contents(), value, buffer->length());
        cpu_modified(buffer);
        return;
    }
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    MTL::CommandBuffer* cmd = queue_->commandBuffer();
    MTL::BlitCommandEncoder* be = cmd->blitCommandEncoder();
    be->fillBuffer(buffer, NS::Range::Make(0, buffer->length()), value);
    be->endEncoding();
    cmd->commit();
    cmd->waitUntilCompleted();
    pool->release();
}

void Context::upload(MTL::Buffer* buffer, std::size_t offset, const void* src, std::size_t bytes) {
    if (bytes == 0) return;
    if (buffer->storageMode() != MTL::StorageModePrivate) {
        std::memcpy(static_cast<std::byte*>(buffer->contents()) + offset, src, bytes);
        cpu_modified(buffer, offset, bytes);
        return;
    }
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    Ref<MTL::Buffer> staging(device_->newBuffer(src, bytes, MTL::ResourceStorageModeShared));
    MTL::CommandBuffer* cmd = queue_->commandBuffer();
    MTL::BlitCommandEncoder* be = cmd->blitCommandEncoder();
    be->copyFromBuffer(staging.get(), 0, buffer, offset, bytes);
    be->endEncoding();
    cmd->commit();
    cmd->waitUntilCompleted();
    pool->release();
}

void Context::download(MTL::Buffer* buffer, std::size_t offset, void* dst, std::size_t bytes) {
    if (bytes == 0) return;
    if (buffer->storageMode() == MTL::StorageModeShared) {
        std::memcpy(dst, static_cast<const std::byte*>(buffer->contents()) + offset, bytes);
        return;
    }
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    Ref<MTL::Buffer> staging(device_->newBuffer(bytes, MTL::ResourceStorageModeShared));
    MTL::CommandBuffer* cmd = queue_->commandBuffer();
    MTL::BlitCommandEncoder* be = cmd->blitCommandEncoder();
    be->copyFromBuffer(buffer, offset, staging.get(), 0, bytes);
    be->endEncoding();
    cmd->commit();
    cmd->waitUntilCompleted();
    std::memcpy(dst, staging->contents(), bytes);
    pool->release();
}

}  // namespace einstar::gpu
