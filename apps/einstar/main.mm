// Einstar desktop app: GLFW window + CAMetalLayer, Dear ImGui panels, einstar::render live view.

#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>
#import <ImageIO/ImageIO.h>
#import <CoreGraphics/CoreGraphics.h>

#include <Metal/Metal.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <cstdlib>
#include <cstring>
#include <string_view>
#include <vector>
#include <filesystem>
#include <format>
#include <memory>
#include <print>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_metal.h"

#include "app_state.hpp"
#include "workflow.hpp"
#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/render/scene_renderer.hpp"

using namespace einstar;

namespace {

struct PreviewTexture {
    id<MTLTexture> texture = nil;
    int width = 0, height = 0;

    // GPU backend: the pipeline's texture is drawn as is (no CPU copy, no upload).
    void set(const gpu::Ref<MTL::Texture>& t) {
        if (!t) return;
        texture = (__bridge id<MTLTexture>)t.get();
        width = static_cast<int>(t->width());
        height = static_cast<int>(t->height());
    }

    void upload(id<MTLDevice> device, const ImageU8& img) {
        if (img.empty()) return;
        if (!texture || width != img.width() || height != img.height()) {
            MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm
                                                                                        width:NSUInteger(img.width())
                                                                                       height:NSUInteger(img.height())
                                                                                    mipmapped:NO];
            d.swizzle = MTLTextureSwizzleChannelsMake(MTLTextureSwizzleRed, MTLTextureSwizzleRed, MTLTextureSwizzleRed,
                                                      MTLTextureSwizzleOne);
            texture = [device newTextureWithDescriptor:d];
            width = img.width();
            height = img.height();
        }
        [texture replaceRegion:MTLRegionMake2D(0, 0, NSUInteger(width), NSUInteger(height))
                   mipmapLevel:0
                     withBytes:img.data()
                   bytesPerRow:NSUInteger(width)];
    }
};

struct MouseState {
    bool rotating = false, panning = false;
    double last_x = 0, last_y = 0;
};

void draw_distance_bar(int step, int steps) {
    // Mirrors the scanner's 10-step LED bar: green in the middle, amber/red at the ends.
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float w = 16.0f, h = 10.0f, gap = 3.0f;
    for (int i = 0; i < steps; ++i) {
        const int dist_from_mid = std::abs(2 * i - (steps - 1));
        ImU32 on = dist_from_mid <= 3 ? IM_COL32(60, 210, 90, 255)
                   : dist_from_mid <= 6 ? IM_COL32(240, 180, 40, 255) : IM_COL32(230, 60, 60, 255);
        const ImU32 col = i == step ? on : IM_COL32(60, 60, 60, 255);
        const float y = p.y + static_cast<float>(steps - 1 - i) * (h + gap);
        dl->AddRectFilled(ImVec2(p.x, y), ImVec2(p.x + w, y + h), col, 2.0f);
    }
    ImGui::Dummy(ImVec2(w, static_cast<float>(steps) * (h + gap)));
}

}  // namespace

// Writes a BGRA8 texture to PNG (used by --snapshot).
static bool write_png(id<MTLTexture> tex, const char* path) {
    const NSUInteger w = tex.width, h = tex.height;
    std::vector<std::uint8_t> px(w * h * 4);
    [tex getBytes:px.data() bytesPerRow:w * 4 fromRegion:MTLRegionMake2D(0, 0, w, h) mipmapLevel:0];
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
    CGContextRef ctx = CGBitmapContextCreate(px.data(), w, h, 8, w * 4, cs,
                                             static_cast<CGBitmapInfo>(kCGImageAlphaPremultipliedFirst) | static_cast<CGBitmapInfo>(kCGBitmapByteOrder32Little));
    CGImageRef img = CGBitmapContextCreateImage(ctx);
    CFURLRef url = CFURLCreateFromFileSystemRepresentation(nullptr, reinterpret_cast<const UInt8*>(path), static_cast<CFIndex>(std::strlen(path)), false);
    CGImageDestinationRef dst = CGImageDestinationCreateWithURL(url, CFSTR("public.png"), 1, nullptr);
    CGImageDestinationAddImage(dst, img, nullptr);
    const bool ok = CGImageDestinationFinalize(dst);
    CFRelease(dst);
    CFRelease(url);
    CGImageRelease(img);
    CGContextRelease(ctx);
    CGColorSpaceRelease(cs);
    return ok;
}

int main(int argc, char** argv) {
    // --snapshot <out.png> [seconds] [--process] [--raw]: run the emulator scan headless and save one frame
    // of the UI; with --process the scan is stopped after `seconds`, processed, and the snapshot shows the
    // mesh; --raw also records raw IR.
    const char* snapshot_path = nullptr;
    double snapshot_seconds = 8.0;
    bool snapshot_process = false;
    bool snapshot_raw = false;
    bool snapshot_markers = false;
    bool snapshot_idle = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string_view(argv[i]) == "--snapshot" && i + 1 < argc) {
            snapshot_path = argv[i + 1];
            if (i + 2 < argc && argv[i + 2][0] != '-') snapshot_seconds = std::atof(argv[i + 2]);
        }
        if (std::string_view(argv[i]) == "--process") snapshot_process = true;
        if (std::string_view(argv[i]) == "--raw") snapshot_raw = true;
        if (std::string_view(argv[i]) == "--markers") snapshot_markers = true;  // snapshot the marker-capture step
        if (std::string_view(argv[i]) == "--idle") snapshot_idle = true;        // snapshot the start screen (not connected)
    }
    bool snapshot_processing = false;
    if (!glfwInit()) {
        std::println(stderr, "glfwInit failed");
        return 1;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    if (snapshot_path) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1600, 1000, "Einstar", nullptr, nullptr);
    if (!window) return 1;

    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    id<MTLCommandQueue> queue = [device newCommandQueue];
    NSWindow* nswin = glfwGetCocoaWindow(window);
    CAMetalLayer* layer = [CAMetalLayer layer];
    layer.device = device;
    layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
    nswin.contentView.layer = layer;
    nswin.contentView.wantsLayer = YES;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOther(window, true);
    ImGui_ImplMetal_Init(device);

    auto ctx = gpu::Context::create((__bridge MTL::Device*)device);
    if (!ctx) {
        std::println(stderr, "{}", ctx.error().message);
        return 1;
    }
    auto renderer = render::SceneRenderer::create(*ctx, MTL::PixelFormatBGRA8Unorm, MTL::PixelFormatDepth32Float);
    if (!renderer) {
        std::println(stderr, "{}", renderer.error().message);
        return 1;
    }

    app::AppState state;
    render::ViewCamera camera;
    render::RenderSettings settings;
    MouseState mouse;
    PreviewTexture preview_left, preview_right;
    gpu::Ref<MTL::Texture> preview_left_ref, preview_right_ref;
    std::vector<pipeline::LiveUpdate::PreviewMarker> preview_markers;
    app::WorkflowUi workflow;
    id<MTLTexture> depth_tex = nil;
    MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor new];
    pass.colorAttachments[0].clearColor = MTLClearColorMake(0.11, 0.12, 0.14, 1.0);
    pass.colorAttachments[0].loadAction = MTLLoadActionClear;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    pass.depthAttachment.loadAction = MTLLoadActionClear;
    pass.depthAttachment.storeAction = MTLStoreActionDontCare;
    pass.depthAttachment.clearDepth = 1.0;

    id<MTLTexture> offscreen = nil;
    Stopwatch snapshot_clock;
    auto next_view_frame = std::chrono::steady_clock::now();
    struct ViewGpuTime {
        std::atomic<std::int64_t> ns{0};
        std::atomic<int> frames{0};
    };
    auto view_gpu = std::make_shared<ViewGpuTime>();
    if (snapshot_path && !snapshot_idle) {
        // Headless runs record into a temporary directory, never the user's scans.
        const auto tmp = std::filesystem::temp_directory_path() / "einstar_snapshot_scans";
        setenv("EINSTAR_SCAN_DIR", tmp.c_str(), 1);
        state.connect(true);
        state.follow_scanner = true;
        state.set_record_raw_ir(snapshot_raw);
        if (snapshot_markers) {
            workflow.type = app::WorkflowUi::ScanType::global_markers;
            workflow.step = app::WorkflowUi::Step::markers;
            state.set_phase(pipeline::ScanPhase::global_markers);
        } else {
            workflow.step = app::WorkflowUi::Step::scan;
        }
        state.start_scan();
    }
    while (!glfwWindowShouldClose(window)) {
        @autoreleasepool {
            glfwPollEvents();
            int fb_w = 0, fb_h = 0;
            glfwGetFramebufferSize(window, &fb_w, &fb_h);
            if (fb_w == 0 || fb_h == 0) continue;
            layer.drawableSize = CGSizeMake(fb_w, fb_h);
            if (!depth_tex || int(depth_tex.width) != fb_w || int(depth_tex.height) != fb_h) {
                MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                                                                            width:NSUInteger(fb_w)
                                                                                           height:NSUInteger(fb_h)
                                                                                        mipmapped:NO];
                d.usage = MTLTextureUsageRenderTarget;
                d.storageMode = MTLStorageModePrivate;
                depth_tex = [device newTextureWithDescriptor:d];
            }

            // Pull whatever the pipeline produced since last frame into GPU buffers.
            state.update();
            if (auto upd = state.take_render_update()) {
                if (upd->model) (*renderer)->set_model_points(*upd->model);
                if (upd->model_gpu) (*renderer)->set_model_buffer(upd->model_gpu->buffer, upd->model_gpu->count);
                if (upd->frame_gpu) (*renderer)->set_frame_buffer(upd->frame_gpu->buffer, upd->frame_gpu->count);
                else (*renderer)->set_frame_points(upd->frame_points);
                (*renderer)->set_markers(upd->markers);
                (*renderer)->set_lines(upd->lines);
                preview_left.upload(device, upd->ir_left);
                if (upd->ir_left_tex) {
                    // Keep the textures alive while ImGui draws them this frame.
                    preview_left_ref = upd->ir_left_tex;
                    preview_right_ref = upd->ir_right_tex;
                    preview_left.set(preview_left_ref);
                    preview_right.set(preview_right_ref);
                }
                preview_markers = std::move(upd->preview_markers);
                if (upd->mesh) {
                    (*renderer)->set_mesh(upd->mesh->first, upd->mesh->second);
                    // A fresh result is shown as a mesh; the live points stay available.
                    settings.show_mesh = true;
                    settings.show_points = upd->mesh->second.empty();
                }
                preview_right.upload(device, upd->ir_right);
                if (state.follow_scanner && upd->scanner_pose) {
                    camera.target = (*upd->scanner_pose * Eigen::Vector4f(0, 0, 300, 1)).head<3>();
                }
            }

            ImGuiIO& io = ImGui::GetIO();
            // Camera navigation when the mouse is over the 3D view.
            double mx, my;
            glfwGetCursorPos(window, &mx, &my);
            if (!io.WantCaptureMouse) {
                const bool l = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
                const bool r = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ||
                               glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
                if (l && mouse.rotating) camera.orbit(float(mx - mouse.last_x) * 0.008f, float(my - mouse.last_y) * 0.008f);
                if (r && mouse.panning) camera.pan(float(mx - mouse.last_x), float(my - mouse.last_y), float(fb_h) / io.DisplayFramebufferScale.y);
                mouse.rotating = l;
                mouse.panning = r;
                if (io.MouseWheel != 0.0f) camera.zoom(std::pow(0.9f, io.MouseWheel));
            } else {
                mouse.rotating = mouse.panning = false;
            }
            mouse.last_x = mx;
            mouse.last_y = my;

            id<CAMetalDrawable> drawable = nil;
            if (snapshot_path) {
                if (!offscreen || int(offscreen.width) != fb_w || int(offscreen.height) != fb_h) {
                    MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                                                                width:NSUInteger(fb_w)
                                                                                               height:NSUInteger(fb_h)
                                                                                            mipmapped:NO];
                    d.usage = MTLTextureUsageRenderTarget;
                    // Shared textures need unified memory; a discrete GPU reads back through a managed copy.
                    d.storageMode = device.hasUnifiedMemory ? MTLStorageModeShared : MTLStorageModeManaged;
                    offscreen = [device newTextureWithDescriptor:d];
                }
                pass.colorAttachments[0].texture = offscreen;
            } else {
                drawable = [layer nextDrawable];
                if (!drawable) continue;
                pass.colorAttachments[0].texture = drawable.texture;
            }
            pass.depthAttachment.texture = depth_tex;
            id<MTLCommandBuffer> cmd = [queue commandBuffer];
            id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:pass];

            (*renderer)->encode((__bridge MTL::RenderCommandEncoder*)enc, camera, float(fb_w), float(fb_h), settings);

            ImGui_ImplMetal_NewFrame(pass);
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            // ---- Workflow panel: connect, scan type, (markers), scan, process ----
            ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(360, 720), ImGuiCond_FirstUseEver);
            ImGui::Begin("Einstar");
            app::draw_workflow(state, workflow, settings);
            ImGui::Separator();
            if (ImGui::CollapsingHeader("Scanner settings")) {
                ImGui::BeginDisabled(!state.connected());
                int level = state.settings.brightness + 1;  // shown 1..21 like the scanner's buttons step it
                if (ImGui::SliderInt("Brightness", &level, 1, device::kBrightnessLevels)) state.set_brightness(level - 1);
                bool raw = state.record_raw_ir();
                if (ImGui::Checkbox("Keep raw IR images (~25 MB/s)", &raw)) state.set_record_raw_ir(raw);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Records the cameras' raw images too, so future depth and marker algorithms can re-run "
                                      "on this scan. Needs a lot of disk space.");
                if (ImGui::TreeNode("Advanced")) {
                    bool changed = false;
                    changed |= ImGui::SliderInt("Exposure", &state.settings.exposure, 500, 12000);
                    changed |= ImGui::SliderInt("Gain", &state.settings.gain, 16, 400);
                    changed |= ImGui::SliderInt("Laser %", &state.settings.laser_percent, 0, 100);
                    changed |= ImGui::SliderInt("Strobe", &state.settings.strobe, 0, 9000);
                    if (changed) state.apply_settings();
                    ImGui::TreePop();
                }
                ImGui::EndDisabled();
            }
            if (ImGui::CollapsingHeader("View")) {
                ImGui::Checkbox("Follow scanner", &state.follow_scanner);
                ImGui::SliderFloat("Point size (mm)", &settings.point_size_mm, 0.1f, 3.0f);
                ImGui::Checkbox("Lighting", &settings.lighting);
                ImGui::TextDisabled("Left-drag: orbit  Right-drag: pan  Wheel: zoom");
            }
            ImGui::End();
            app::draw_status_banner(state, workflow);

            // ---- HUD ----
            const auto hud = state.hud();
            if (state.connected()) {
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 250, 10), ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.6f);
            ImGui::Begin("HUD", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove);
            if (!state.scanning()) {
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1), "%s", hud.frames > 0 ? "Paused" : "Idle");
            } else if (hud.tracking_lost) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "TRACKING LOST");
                ImGui::TextUnformatted("Move back to the grey frustum");
                if (!hud.reason.empty()) ImGui::TextDisabled("%s", hud.reason.c_str());
            } else {
                ImGui::TextColored(ImVec4(0.3f, 1, 0.4f, 1), "Tracking");
            }
            if (!hud.notice.empty()) ImGui::TextColored(ImVec4(1, 0.85f, 0.3f, 1), "%s", hud.notice.c_str());
            ImGui::Text("FPS            %5.1f", hud.fps);
            ImGui::Text("Frames         %5d", hud.frames);
            ImGui::Text("Model points   %zu", hud.model_points);
            if (hud.phase == pipeline::ScanPhase::global_markers) ImGui::TextColored(ImVec4(1, 0.75f, 0.1f, 1), "GLOBAL MARKER CAPTURE");
            ImGui::Text("Markers        %2d / %2d seen", hud.markers_matched, hud.markers);
            ImGui::Text("Marker map     %5d%s", hud.map_markers, hud.global_markers > 0 ? " (global)" : "");
            ImGui::Text("Point distance %4.2f mm", hud.point_distance_mm);
            ImGui::Text("Depth / track  %5.1f / %5.1f ms", hud.depth_ms, hud.track_ms);
            ImGui::Text("Queue / drops  %d / %d", hud.queue_depth, hud.dropped);
            if (hud.recorded_frames > 0) ImGui::Text("Recorded       %llu frames", static_cast<unsigned long long>(hud.recorded_frames));
            if (hud.raw_frames > 0 || hud.raw_dropped > 0)
                ImGui::Text("Raw IR         %llu frames%s", static_cast<unsigned long long>(hud.raw_frames),
                            hud.raw_dropped > 0 ? std::format(" ({} not written)", hud.raw_dropped).c_str() : "");
            if (hud.temperature_c > -100) ImGui::Text("Temperature    %4.1f C", hud.temperature_c);
            ImGui::Separator();
            ImGui::Text("Distance %s", hud.distance_mm > 0 ? std::format("{:.0f} mm", hud.distance_mm).c_str() : "--");
            draw_distance_bar(hud.distance_step, 10);
            ImGui::End();
            }

            // ---- Camera previews ----
            if (preview_left.texture) {
                ImGui::SetNextWindowPos(ImVec2(10, io.DisplaySize.y - 230), ImGuiCond_FirstUseEver);
                ImGui::Begin("Cameras", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
                const float pw = 256.0f, ph = pw * float(preview_left.height) / float(preview_left.width);
                ImGui::Image((ImTextureID)(__bridge void*)preview_left.texture, ImVec2(pw, ph));
                {
                    // Marker detections on the left preview: green = stereo matched, red = left only.
                    const ImVec2 o = ImGui::GetItemRectMin();
                    const float k = pw / float(preview_left.width);
                    auto* dl = ImGui::GetWindowDrawList();
                    for (const auto& m : preview_markers)
                        dl->AddCircle(ImVec2(o.x + m.x * k, o.y + m.y * k), std::max(3.0f, m.radius * k + 1.5f),
                                      m.matched ? IM_COL32(40, 230, 90, 255) : IM_COL32(230, 60, 60, 255), 0, 1.5f);
                }
                ImGui::SameLine();
                if (preview_right.texture) ImGui::Image((ImTextureID)(__bridge void*)preview_right.texture, ImVec2(pw, ph));
                ImGui::End();
            }

            ImGui::Render();
            ImGui_ImplMetal_RenderDrawData(ImGui::GetDrawData(), cmd, enc);
            [enc endEncoding];
            if (offscreen && !drawable && offscreen.storageMode == MTLStorageModeManaged) {
                id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
                [blit synchronizeResource:offscreen];
                [blit endEncoding];
            }
            if (drawable) [cmd presentDrawable:drawable];
            if (snapshot_path) {
                // GPU time of the view (reported with the snapshot: it shares the GPU with the scan).
                [cmd addCompletedHandler:^(id<MTLCommandBuffer> c) {
                    view_gpu->ns.fetch_add(static_cast<std::int64_t>((c.GPUEndTime - c.GPUStartTime) * 1e9));
                    view_gpu->frames.fetch_add(1);
                }];
            }
            [cmd commit];
            if (snapshot_path) {
                // Headless: no display to pace the loop, so pace it like a 60 Hz display (unpaced, it
                // floods the GPU with view renders and starves the scan).
                std::this_thread::sleep_until(next_view_frame);
                next_view_frame = std::max(next_view_frame + std::chrono::microseconds(16667), std::chrono::steady_clock::now());
            }
            if (snapshot_path && snapshot_process && snapshot_clock.elapsed_ms() > snapshot_seconds * 1000.0) {
                if (!snapshot_processing) {
                    state.stop_scan();
                    workflow.step = app::WorkflowUi::Step::process;
                    state.follow_scanner = false;
                    recon::ProcessParams pp;
                    state.process_scan(pp);
                    snapshot_processing = true;
                }
                const auto ps = state.process_status();
                if (ps.running || !ps.done) {
                    if (!ps.running && !ps.done) {
                        std::println("process failed: {}", ps.summary);
                        break;
                    }
                    continue;
                }
                snapshot_process = false;  // done: take the snapshot on the next frame
                snapshot_seconds = 0;
                std::println("process: {}", ps.summary);
                continue;
            }
            if (snapshot_path && !snapshot_process && snapshot_clock.elapsed_ms() > snapshot_seconds * 1000.0) {
                [cmd waitUntilCompleted];
                const auto st = state.hud();
                std::println("view: {:.2f} ms GPU per frame over {} frames; scan: {:.1f} fps, depth {:.1f} ms, track {:.1f} ms, "
                             "{} frames, queue {}, {} dropped",
                             view_gpu->frames.load() ? static_cast<double>(view_gpu->ns.load()) * 1e-6 / view_gpu->frames.load() : 0.0,
                             view_gpu->frames.load(), st.fps, st.depth_ms, st.track_ms, st.frames, st.queue_depth, st.dropped);
                std::println("snapshot: {}", write_png(offscreen, snapshot_path) ? snapshot_path : "FAILED");
                break;
            }
        }
    }

    state.stop_scan();
    ImGui_ImplMetal_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
