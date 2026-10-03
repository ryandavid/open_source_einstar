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
#include <optional>
#include <unistd.h>
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
#include "agent_methods.hpp"
#include "einstar/agent/server.hpp"
#include "upright_image.hpp"

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

// Writes a BGRA8 texture to PNG (used by --snapshot). The texture is GPU-private; a blit on `queue`, after the
// frames already queued, copies it into a shared buffer the CPU can read on any GPU.
static bool write_png(id<MTLCommandQueue> queue, id<MTLTexture> tex, const char* path) {
    const NSUInteger w = tex.width, h = tex.height;
    id<MTLBuffer> px = [queue.device newBufferWithLength:w * h * 4 options:MTLResourceStorageModeShared];
    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
    [blit copyFromTexture:tex sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0) sourceSize:MTLSizeMake(w, h, 1)
                 toBuffer:px destinationOffset:0 destinationBytesPerRow:w * 4 destinationBytesPerImage:w * h * 4];
    [blit endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];
    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
    CGContextRef ctx = CGBitmapContextCreate(px.contents, w, h, 8, w * 4, cs,
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
    // --snapshot <out.png> [seconds] [--process] [--raw] [--edit select|delete]: run the emulator scan
    // headless and save one frame of the UI; with --process the scan is stopped after `seconds`, processed,
    // and the snapshot shows the mesh; --raw also records raw IR; --edit pauses after `seconds` and lassos
    // a block left of the view's centre (and deletes it), before any processing.
    const char* snapshot_path = nullptr;
    double snapshot_seconds = 8.0;
    bool snapshot_process = false;
    bool snapshot_raw = false;
    bool snapshot_markers = false;
    bool snapshot_idle = false;
    int snapshot_edit = 0;  // 1 select, 2 delete
    // --mcp[=<socket>] [--visible]: agent control (libs/agent, einstar-mcp). The window is hidden unless
    // --visible; scans record into a temporary folder unless EINSTAR_SCAN_DIR says otherwise.
    std::optional<std::string> mcp_socket;
    bool mcp_visible = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--mcp") mcp_socket = std::format("/tmp/einstar_scan_{}.sock", ::getpid());
        if (a.starts_with("--mcp=")) mcp_socket = std::string(a.substr(6));
        if (a == "--visible") mcp_visible = true;
        if (std::string_view(argv[i]) == "--edit" && i + 1 < argc) snapshot_edit = std::string_view(argv[i + 1]) == "delete" ? 2 : 1;
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
    int snapshot_edit_stage = 0;
    if (!glfwInit()) {
        std::println(stderr, "glfwInit failed");
        return 1;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    // Headless (a snapshot, or an agent without --visible): rendered offscreen, paced like a display.
    const bool offscreen_mode = snapshot_path || (mcp_socket && !mcp_visible);
    if (offscreen_mode) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    if (mcp_socket && !std::getenv("EINSTAR_SCAN_DIR"))
        setenv("EINSTAR_SCAN_DIR", (std::filesystem::temp_directory_path() / "einstar_agent_scans").c_str(), 1);
    GLFWwindow* window = glfwCreateWindow(1600, 1000, "Einstar", nullptr, nullptr);
    if (!window) return 1;

    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    id<MTLCommandQueue> queue = [device newCommandQueue];
    NSWindow* nswin = glfwGetCocoaWindow(window);
    CAMetalLayer* layer = [CAMetalLayer layer];
    layer.device = device;
    layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
    if (mcp_socket) layer.framebufferOnly = NO;  // screenshots copy the drawable
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
    std::optional<Eigen::Matrix4f> scanner_pose;  // latest: the scanner model, "Follow scanner"
    std::uint64_t shown_selection = 0;            // the edit selection the renderer has
    // A lasso being drawn (Shift-drag adds, Option-drag removes), in window points.
    struct Lasso {
        bool active = false, subtract = false;
        std::vector<ImVec2> points;
    } lasso;
    auto last_view_frame = std::chrono::steady_clock::now();
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
    std::unique_ptr<agent::Server> agent;
    if (mcp_socket) {
        agent = std::make_unique<agent::Server>(agent::App::scan, "scan");
        agent->set_visible(mcp_visible);
        app::register_scan_agent(*agent, {state, workflow, camera, settings});
        std::string why;
        if (!agent->start(*mcp_socket, why)) {
            std::println(stderr, "--mcp: {}", why);
            return 1;
        }
        std::println("AGENT_READY {} {}", *mcp_socket, ::getpid());
        std::fflush(stdout);
    }
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
    while (!glfwWindowShouldClose(window) && !(agent && agent->quit_requested())) {
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
            auto upd = state.take_render_update();
            if (upd) {
                if (upd->model) (*renderer)->set_model_points(*upd->model);
                if (upd->model_gpu) (*renderer)->set_model_buffer(upd->model_gpu->buffer, upd->model_gpu->count);
            }
            if (upd && upd->model_only) {
                (*renderer)->set_frame_points({});  // an edit: the last frame's overlay is stale
            } else if (upd) {
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
                if (upd->scanner_pose) scanner_pose = upd->scanner_pose;
            }
            if (state.selection_version() != shown_selection) {
                (*renderer)->set_selection(state.selection());
                shown_selection = state.selection_version();
            }

            ImGuiIO& io = ImGui::GetIO();
            // Camera navigation when the mouse is over the 3D view.
            // (ImGui's input, not GLFW's: an agent's synthetic input arrives there too.)
            const bool mouse_valid = ImGui::IsMousePosValid(&io.MousePos);
            const double mx = mouse_valid ? io.MousePos.x : mouse.last_x, my = mouse_valid ? io.MousePos.y : mouse.last_y;
            // Lasso (a paused scan): Shift-drag adds to the selection, Option-drag removes from it; the
            // drag draws instead of orbiting. On release the outline, in framebuffer px with this view's
            // projection, becomes a stroke.
            {
                const bool l_down = io.MouseDown[0];
                const ImVec2 at(static_cast<float>(mx), static_cast<float>(my));
                if (lasso.active && l_down) {
                    const ImVec2 last = lasso.points.back();
                    if (std::hypot(at.x - last.x, at.y - last.y) >= 3.0f) lasso.points.push_back(at);
                } else if (lasso.active) {
                    LassoStroke stroke;
                    stroke.view_proj = camera.projection(static_cast<float>(fb_w) / static_cast<float>(std::max(fb_h, 1))) * camera.view();
                    stroke.viewport = Eigen::Vector2f(static_cast<float>(fb_w), static_cast<float>(fb_h));
                    stroke.subtract = lasso.subtract;
                    for (const auto& q : lasso.points)
                        stroke.polygon.emplace_back(q.x * io.DisplayFramebufferScale.x, q.y * io.DisplayFramebufferScale.y);
                    state.add_lasso(std::move(stroke));
                    lasso = {};
                } else if (!io.WantCaptureMouse && l_down && !mouse.rotating && (io.KeyShift || io.KeyAlt) && state.can_edit()) {
                    lasso.active = true;
                    lasso.subtract = io.KeyAlt && !io.KeyShift;
                    lasso.points = {at};
                }
            }
            // (Not while typing in a text field; a focused panel is fine.)
            if (!io.WantTextInput) {
                if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
                    if (lasso.active) lasso = {};
                    else state.clear_selection();
                }
                if (ImGui::IsKeyPressed(ImGuiKey_Delete, false) || ImGui::IsKeyPressed(ImGuiKey_Backspace, false)) state.delete_selection();
                if (io.KeySuper && ImGui::IsKeyPressed(ImGuiKey_Z, false)) state.undo_delete();
            }
            if (lasso.active && lasso.points.size() > 1) {
                // The outline so far, and (fainter) the edge that will close it.
                auto* fg = ImGui::GetForegroundDrawList();
                const ImU32 col = lasso.subtract ? IM_COL32(110, 190, 255, 255) : IM_COL32(255, 120, 70, 255);
                fg->AddPolyline(lasso.points.data(), static_cast<int>(lasso.points.size()), col, ImDrawFlags_None, 2.0f);
                fg->AddLine(lasso.points.back(), lasso.points.front(), (col & 0x00FFFFFFu) | (90u << IM_COL32_A_SHIFT), 1.0f);
            }
            if (!io.WantCaptureMouse) {
                const bool l = io.MouseDown[0];
                const bool r = io.MouseDown[1] || io.MouseDown[2];
                const bool moved = mx != mouse.last_x || my != mouse.last_y;
                // Turning or moving the view by hand leaves "Follow scanner"; zooming keeps it.
                if (l && mouse.rotating && moved && !lasso.active) {
                    camera.orbit(float(mx - mouse.last_x) * 0.008f, float(my - mouse.last_y) * 0.008f);
                    state.follow_scanner = false;
                }
                if (r && mouse.panning && moved) {
                    camera.pan(float(mx - mouse.last_x), float(my - mouse.last_y), float(fb_h) / io.DisplayFramebufferScale.y);
                    state.follow_scanner = false;
                }
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && !io.KeyShift && !io.KeyAlt) {
                    camera.reset();
                    state.follow_scanner = false;
                }
                mouse.rotating = l;
                mouse.panning = r;
                if (io.MouseWheel != 0.0f) camera.zoom(std::pow(0.9f, io.MouseWheel));
            } else {
                mouse.rotating = mouse.panning = false;
            }
            mouse.last_x = mx;
            mouse.last_y = my;

            // Follow: ease towards the view from behind the scanner (poses arrive at the scan rate, the
            // view redraws faster; ~80 ms time constant whatever the display rate).
            const auto now = std::chrono::steady_clock::now();
            const double dt_s = std::chrono::duration<double>(now - last_view_frame).count();
            last_view_frame = now;
            if (!state.connected()) scanner_pose.reset();
            if (state.follow_scanner && scanner_pose)
                camera.follow(*scanner_pose, static_cast<float>(1.0 - std::exp(-dt_s / 0.08)));
            (*renderer)->set_scanner_pose(scanner_pose);

            id<CAMetalDrawable> drawable = nil;
            if (offscreen_mode) {
                if (!offscreen || int(offscreen.width) != fb_w || int(offscreen.height) != fb_h) {
                    MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                                                                width:NSUInteger(fb_w)
                                                                                               height:NSUInteger(fb_h)
                                                                                            mipmapped:NO];
                    d.usage = MTLTextureUsageRenderTarget;
                    d.storageMode = MTLStorageModePrivate;  // read back by write_png
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
            if (agent) agent->pump();  // the agent's requests and synthetic input, before the frame
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
                ImGui::Checkbox("Show scanner", &settings.show_scanner);
                ImGui::TextDisabled("Left-drag: orbit  Right-drag: pan  Wheel: zoom  Double-click: reset view");
                ImGui::TextDisabled("Follow: the view from behind and above the scanner; orbiting or panning leaves it");
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
            else if (!hud.temperature_note.empty()) ImGui::TextDisabled("Temperature    n/a (%s)", hud.temperature_note.c_str());
            ImGui::Separator();
            ImGui::Text("Distance %s", hud.distance_mm > 0 ? std::format("{:.0f} mm", hud.distance_mm).c_str() : "--");
            draw_distance_bar(hud.distance_step, 10);
            ImGui::End();
            }

            // ---- Camera previews ----
            if (preview_left.texture) {
                ImGui::SetNextWindowPos(ImVec2(10, io.DisplaySize.y - 290), ImGuiCond_FirstUseEver);
                ImGui::Begin("Cameras", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
                // Upright, as the scanner is held (upright_image.hpp): left and right side by side.
                constexpr float kPreviewWidth = 205.0f;  // drawn; taller than wide
                auto* dl = ImGui::GetWindowDrawList();
                const auto label = [&](const app::ImageFrame& f, const char* s) {
                    dl->AddText(ImVec2(f.origin.x + 4, f.origin.y + 2), IM_COL32(220, 220, 220, 220), s);
                };
                const auto left = app::upright_image_item((ImTextureID)(__bridge void*)preview_left.texture, float(preview_left.width),
                                                          float(preview_left.height), kPreviewWidth);
                // Marker detections on the left preview: green = stereo matched, red = left only.
                for (const auto& m : preview_markers)
                    dl->AddCircle(left.at(m.x, m.y), std::max(3.0f, m.radius * left.scale + 1.5f),
                                  m.matched ? IM_COL32(40, 230, 90, 255) : IM_COL32(230, 60, 60, 255), 0, 1.5f);
                label(left, "Left");
                if (preview_right.texture) {
                    ImGui::SameLine();
                    label(app::upright_image_item((ImTextureID)(__bridge void*)preview_right.texture, float(preview_right.width),
                                                  float(preview_right.height), kPreviewWidth),
                          "Right");
                }
                // Keep the panel on screen (a saved layout, a smaller window).
                const ImVec2 wp = ImGui::GetWindowPos(), ws = ImGui::GetWindowSize();
                if (wp.y + ws.y > io.DisplaySize.y) ImGui::SetWindowPos(ImVec2(wp.x, std::max(0.0f, io.DisplaySize.y - ws.y)));
                ImGui::End();
            }

            ImGui::Render();
            ImGui_ImplMetal_RenderDrawData(ImGui::GetDrawData(), cmd, enc);
            [enc endEncoding];
            // A screenshot for the agent: this frame's pixels, copied before it is presented.
            id<MTLBuffer> capture_buf = nil;
            id<MTLTexture> target_tex = pass.colorAttachments[0].texture;
            if (agent && agent->capture_wanted()) {
                capture_buf = [device newBufferWithLength:target_tex.width * target_tex.height * 4 options:MTLResourceStorageModeShared];
                id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
                [blit copyFromTexture:target_tex sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0)
                           sourceSize:MTLSizeMake(target_tex.width, target_tex.height, 1) toBuffer:capture_buf destinationOffset:0
                destinationBytesPerRow:target_tex.width * 4 destinationBytesPerImage:target_tex.width * target_tex.height * 4];
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
            if (capture_buf) {
                [cmd waitUntilCompleted];
                agent::Frame frame;
                frame.width = static_cast<int>(target_tex.width);
                frame.height = static_cast<int>(target_tex.height);
                frame.scale = io.DisplayFramebufferScale.x;
                const auto* bytes = static_cast<const std::uint8_t*>(capture_buf.contents);
                frame.bgra.assign(bytes, bytes + capture_buf.length);
                agent->provide_capture(std::move(frame));
            }
            if (offscreen_mode) {
                // Headless: no display to pace the loop, so pace it like a 60 Hz display (unpaced, it
                // floods the GPU with view renders and starves the scan).
                std::this_thread::sleep_until(next_view_frame);
                next_view_frame = std::max(next_view_frame + std::chrono::microseconds(16667), std::chrono::steady_clock::now());
            }
            if (snapshot_path && snapshot_edit != 0 && snapshot_clock.elapsed_ms() > snapshot_seconds * 1000.0) {
                // Pause, lasso a block of the view (as a Shift-drag would), delete it if asked, then carry on.
                if (snapshot_edit_stage == 0) {
                    state.stop_scan();
                    snapshot_edit_stage = 1;
                } else if (snapshot_edit_stage == 1 && state.can_edit()) {
                    LassoStroke stroke;
                    stroke.view_proj = camera.projection(static_cast<float>(fb_w) / static_cast<float>(std::max(fb_h, 1))) * camera.view();
                    stroke.viewport = Eigen::Vector2f(static_cast<float>(fb_w), static_cast<float>(fb_h));
                    const float x0 = 0.30f * static_cast<float>(fb_w), x1 = 0.48f * static_cast<float>(fb_w);
                    const float y0 = 0.25f * static_cast<float>(fb_h), y1 = 0.60f * static_cast<float>(fb_h);
                    stroke.polygon = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
                    state.add_lasso(std::move(stroke));
                    if (snapshot_edit == 2) state.delete_selection();
                    snapshot_edit_stage = 2;
                } else if (snapshot_edit_stage == 2) {
                    const auto es = state.edit_status();
                    if (snapshot_edit == 1 || (!es.busy && es.undo_depth > 0)) {
                        if (!es.message.empty()) std::println("edit: {}", es.message);
                        snapshot_edit = 0;  // done: render the result, then snapshot or process
                    }
                }
                continue;
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
                std::println("snapshot: {}", write_png(queue, offscreen, snapshot_path) ? snapshot_path : "FAILED");
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
