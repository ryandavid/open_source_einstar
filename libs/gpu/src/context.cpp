#include "einstar/gpu/context.hpp"

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
    NS::AutoreleasePool* pool = NS::AutoreleasePool::alloc()->init();
    Ref<MTL::Function> fn(lib->newFunction(ns_string(function)));
    if (!fn) {
        pool->release();
        return make_error(Errc::not_found, std::format("Metal function '{}' not found", function));
    }
    NS::Error* err = nullptr;
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

}  // namespace einstar::gpu
