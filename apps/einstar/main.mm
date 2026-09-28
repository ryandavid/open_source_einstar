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

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string_view>
#include <vector>
#include <format>
#include <memory>
#include <print>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_metal.h"

#include "app_state.hpp"
#include "einstar/core/log.hpp"
#include "einstar/core/timing.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/render/scene_renderer.hpp"

using namespace einstar;

namespace {

struct PreviewTexture {
    id<MTLTexture> texture = nil;
    int width = 0, height = 0;

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
    // --snapshot <out.png> [seconds]: run the emulator scan headless and save one frame of the UI.
    const char* snapshot_path = nullptr;
    double snapshot_seconds = 8.0;
    for (int i = 1; i < argc; ++i)
        if (std::string_view(argv[i]) == "--snapshot" && i + 1 < argc) {
            snapshot_path = argv[i + 1];
            if (i + 2 < argc) snapshot_seconds = std::atof(argv[i + 2]);
        }
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
    std::vector<pipeline::LiveUpdate::PreviewMarker> preview_markers;
    int align_mode = 1;  // hybrid
    char marker_path[512] = {};
    if (const char* home = std::getenv("HOME")) std::snprintf(marker_path, sizeof marker_path, "%s/Documents/einstar_global_markers.txt", home);
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
    if (snapshot_path) {
        state.connect(true);
        state.follow_scanner = true;
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
                preview_markers = std::move(upd->preview_markers);
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
                    d.storageMode = MTLStorageModeShared;
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

            // ---- Control panel ----
            ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(340, 470), ImGuiCond_FirstUseEver);
            ImGui::Begin("Scan");
            ImGui::TextWrapped("%s", state.status().c_str());
            if (!state.scanning()) {
                if (ImGui::Button("Connect scanner")) state.connect(false);
                ImGui::SameLine();
                if (ImGui::Button("Use emulator")) state.connect(true);
            }
            ImGui::Separator();
            ImGui::BeginDisabled(!state.connected());
            if (!state.scanning()) {
                if (ImGui::Button("Start scan", ImVec2(-1, 0))) state.start_scan();
            } else {
                if (ImGui::Button("Stop scan", ImVec2(-1, 0))) state.stop_scan();
            }
            if (ImGui::Button("Clear model", ImVec2(-1, 0))) state.clear_model();
            ImGui::EndDisabled();
            ImGui::Separator();
            ImGui::TextUnformatted("Alignment");
            if (ImGui::Combo("##align", &align_mode, "Geometry\0Hybrid (surface + markers)\0Markers\0"))
                state.set_align_mode(align_mode == 0 ? track::AlignMode::geometry : align_mode == 2 ? track::AlignMode::markers : track::AlignMode::hybrid);
            {
                const auto hud_now = state.hud();
                ImGui::BeginDisabled(!state.connected());
                if (ImGui::CollapsingHeader("Global markers", ImGuiTreeNodeFlags_DefaultOpen)) {
                    const bool capturing = hud_now.phase == pipeline::ScanPhase::global_markers;
                    ImGui::TextWrapped(capturing ? "Capturing the marker constellation (no surface is recorded). Sweep over every marker, "
                                                   "then optimise."
                                                 : "Scan markers first for a drift-free frame for the whole object.");
                    if (!capturing) {
                        if (ImGui::Button("Capture markers", ImVec2(-1, 0))) state.set_phase(pipeline::ScanPhase::global_markers);
                    } else {
                        ImGui::Text("Keyframes %d, markers %d", hud_now.keyframes, hud_now.map_markers);
                        if (ImGui::Button("Optimise and use", ImVec2(-1, 0))) {
                            state.optimize_global_markers();
                            state.set_phase(pipeline::ScanPhase::surface);
                        }
                        if (ImGui::Button("Back to surface scan", ImVec2(-1, 0))) state.set_phase(pipeline::ScanPhase::surface);
                    }
                    if (hud_now.global_markers > 0) {
                        ImGui::TextColored(ImVec4(1, 0.75f, 0.1f, 1), "%d global markers in use", hud_now.global_markers);
                        if (ImGui::Button("Discard global markers", ImVec2(-1, 0))) state.clear_global_markers();
                    }
                    ImGui::InputText("##path", marker_path, sizeof marker_path);
                    if (ImGui::Button("Save")) (void)state.save_global_markers(marker_path);
                    ImGui::SameLine();
                    if (ImGui::Button("Load")) (void)state.load_global_markers(marker_path);
                    if (const auto st = state.global_marker_status(); !st.empty()) ImGui::TextWrapped("%s", st.c_str());
                }
                ImGui::EndDisabled();
            }
            ImGui::Separator();
            ImGui::TextUnformatted("Scanner settings");
            bool changed = false;
            changed |= ImGui::SliderInt("Exposure", &state.settings.exposure, 500, 12000);
            changed |= ImGui::SliderInt("Gain", &state.settings.gain, 16, 400);
            changed |= ImGui::SliderInt("Laser %", &state.settings.laser_percent, 0, 100);
            changed |= ImGui::SliderInt("Strobe", &state.settings.strobe, 0, 9000);
            if (changed) state.apply_settings();
            ImGui::Separator();
            ImGui::Checkbox("Follow scanner", &state.follow_scanner);
            ImGui::SliderFloat("Point size (mm)", &settings.point_size_mm, 0.1f, 3.0f);
            ImGui::Checkbox("Lighting", &settings.lighting);
            ImGui::Separator();
            ImGui::TextUnformatted("Left-drag: orbit  Right-drag: pan  Wheel: zoom");
            ImGui::TextUnformatted("Scanner button: start / stop");
            ImGui::End();

            // ---- HUD ----
            const auto hud = state.hud();
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 250, 10), ImGuiCond_Always);
            ImGui::SetNextWindowBgAlpha(0.6f);
            ImGui::Begin("HUD", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove);
            if (!state.scanning()) {
                ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1), "Idle");
            } else if (hud.tracking_lost) {
                ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "TRACKING LOST");
                ImGui::TextUnformatted("Move back to the grey frustum");
                if (!hud.reason.empty()) ImGui::TextDisabled("%s", hud.reason.c_str());
            } else {
                ImGui::TextColored(ImVec4(0.3f, 1, 0.4f, 1), "Tracking");
            }
            ImGui::Text("FPS            %5.1f", hud.fps);
            ImGui::Text("Frames         %5d", hud.frames);
            ImGui::Text("Model points   %zu", hud.model_points);
            if (hud.phase == pipeline::ScanPhase::global_markers) ImGui::TextColored(ImVec4(1, 0.75f, 0.1f, 1), "GLOBAL MARKER CAPTURE");
            ImGui::Text("Markers        %2d / %2d seen", hud.markers_matched, hud.markers);
            ImGui::Text("Marker map     %5d%s", hud.map_markers, hud.global_markers > 0 ? " (global)" : "");
            ImGui::Text("Point distance %4.2f mm", hud.point_distance_mm);
            ImGui::Text("Depth / track  %5.1f / %5.1f ms", hud.depth_ms, hud.track_ms);
            ImGui::Text("Queue / drops  %d / %d", hud.queue_depth, hud.dropped);
            if (hud.temperature_c > -100) ImGui::Text("Temperature    %4.1f C", hud.temperature_c);
            ImGui::Separator();
            ImGui::Text("Distance %s", hud.distance_mm > 0 ? std::format("{:.0f} mm", hud.distance_mm).c_str() : "--");
            draw_distance_bar(hud.distance_step, 10);
            ImGui::End();

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
            if (drawable) [cmd presentDrawable:drawable];
            [cmd commit];
            if (snapshot_path && snapshot_clock.elapsed_ms() > snapshot_seconds * 1000.0) {
                [cmd waitUntilCompleted];
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
