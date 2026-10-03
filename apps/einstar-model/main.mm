// Einstar Model: scan -> labelled faces, holes and fillets -> constraints -> solid -> STEP.
// GLFW window + CAMetalLayer, Dear ImGui panels, the scan or model drawn by einstar::render.
//
//   EinstarModel [file]                    a scan (.estr, .stl, .ply) or a model (.emodel) to open
//   EinstarModel --mcp[=<socket>] [--visible]   under agent control (libs/agent, einstar-mcp)
//   EinstarModel --snapshot <out.png>      the demo part detected, solved and built, one frame saved

#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#import <CoreGraphics/CoreGraphics.h>
#import <ImageIO/ImageIO.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <Metal/Metal.hpp>

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <format>
#include <optional>
#include <print>
#include <string>
#include <thread>
#include <unistd.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_metal.h"

#include "agent_methods.hpp"
#include "dialogs.hpp"
#include "einstar/agent/server.hpp"
#include "einstar/gpu/context.hpp"
#include "einstar/model/settings.hpp"
#include "einstar/render/scene_renderer.hpp"
#include "model_app.hpp"
#include "photo_ui.hpp"

using namespace einstar;

namespace {

// Files dropped on the window, taken by the main loop.
std::vector<std::filesystem::path> g_dropped;
void on_drop(GLFWwindow*, int count, const char** paths) {
    for (int i = 0; i < count; ++i) g_dropped.emplace_back(paths[i]);
}

bool is_photo(const std::filesystem::path& p) {
    std::string ext = p.extension().string();
    if (!ext.empty()) ext.erase(0, 1);
    for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return std::ranges::find(modelapp::photo_extensions(), ext) != modelapp::photo_extensions().end();
}

// The world ray under a window point.
std::pair<Vec3f, Vec3f> ray_at(const render::ViewCamera& camera, float x, float y, float w, float h) {
    const Eigen::Matrix4f inv = (camera.projection(w / std::max(h, 1.0f)) * camera.view()).inverse();
    const float nx = 2 * x / w - 1, ny = 1 - 2 * y / h;
    Eigen::Vector4f a = inv * Eigen::Vector4f(nx, ny, 0, 1), b = inv * Eigen::Vector4f(nx, ny, 1, 1);
    const Vec3f near = a.head<3>() / a.w(), far = b.head<3>() / b.w();
    return {near, (far - near).normalized()};
}

bool write_png(id<MTLCommandQueue> queue, id<MTLTexture> tex, const char* path) {
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

}  // namespace

int main(int argc, char** argv) {
    std::optional<std::string> mcp_socket, open_path;
    const char* snapshot_path = nullptr;
    bool mcp_visible = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--mcp") mcp_socket = std::format("/tmp/einstar_model_{}.sock", ::getpid());
        else if (a.starts_with("--mcp=")) mcp_socket = std::string(a.substr(6));
        else if (a == "--visible") mcp_visible = true;
        else if (a == "--snapshot" && i + 1 < argc) snapshot_path = argv[++i];
        else if (!a.starts_with("-")) open_path = std::string(a);
    }
    if (!glfwInit()) {
        std::println(stderr, "glfwInit failed");
        return 1;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    const bool offscreen_mode = snapshot_path || (mcp_socket && !mcp_visible);
    if (offscreen_mode) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(1600, 1000, "Einstar Model", nullptr, nullptr);
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
    // The layout is kept with the settings, not in whatever folder the app was started from.
    static const std::string ini = (model::settings_dir() / "model-imgui.ini").string();
    std::error_code ec;
    std::filesystem::create_directories(model::settings_dir(), ec);
    ImGui::GetIO().IniFilename = offscreen_mode ? nullptr : ini.c_str();
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
    render::RenderSettings rs;
    rs.show_points = false;
    rs.show_scanner = false;

    modelapp::ModelApp app;
    modelapp::PhotoUi photos((__bridge void*)device);
    glfwSetDropCallback(window, on_drop);
    render::ViewCamera camera;
    std::unique_ptr<agent::Server> agent;
    if (mcp_socket) {
        agent = std::make_unique<agent::Server>(agent::App::model, "model");
        agent->set_visible(mcp_visible);
        modelapp::register_model_agent(*agent, app, camera);
        std::string why;
        if (!agent->start(*mcp_socket, why)) {
            std::println(stderr, "--mcp: {}", why);
            return 1;
        }
        std::println("AGENT_READY {} {}", *mcp_socket, ::getpid());
        std::fflush(stdout);
    } else if (!snapshot_path) {
        // Started by the user: an agent (einstar-mcp's app_attach) can work in this window alongside them, through
        // a socket only this user can open. A second instance leaves the first one's socket alone.
        const std::string sock = (model::settings_dir() / "model-agent.sock").string();
        if (modelapp::agent_socket_in_use(sock)) {
            std::println(stderr, "another Einstar Model is open: agents attach to that one");
        } else {
            agent = std::make_unique<agent::Server>(agent::App::model, "model");
            agent->set_visible(true);
            modelapp::register_model_agent(*agent, app, camera);
            std::string why;
            if (!agent->start(sock, why, false)) {
                std::println(stderr, "agent socket: {}", why);
                agent.reset();
            }
        }
    }
    if (open_path) app.open(*open_path);
    if (snapshot_path) {
        // The demo part modelled as a user would: faces detected and named, squared to a datum, the unseen bottom
        // added, the measured fillet radius and blind-hole depth given; then solved, built and its deviation shown.
        app.run("open_demo", {});
        app.run("detect", {});
        const auto label_hit = [&](model::json origin, model::json direction) {
            return app.doc.apply("raycast", {{"origin", origin}, {"direction", direction}}).result.value("label", std::string());
        };
        app.run("label.update", {{"label", label_hit({10, 8, 100}, {0, 0, -1})}, {"name", "top"}});
        app.run("label.update", {{"label", label_hit({80, 3, 12}, {-1, 0, 0})}, {"name", "right"}});
        app.run("datum.create", {{"name", "part"}, {"z", "top"}, {"x", "right"}});
        app.run("square", {{"datum", "part"}});
        app.run("face.add_plane", {{"name", "bottom"}, {"datum", "part"}, {"axis", "z"}, {"offset", -20.0}, {"facing", "-"}});
        const auto summary = app.doc.apply("summary", {}).result;
        for (const auto& f : summary["fillets"]) app.run("fillet.update", {{"fillet", f["name"]}, {"radius", 2.0}});
        for (const auto& h : summary["holes"])
            if (h["measured_diameter"].get<double>() > 7) app.run("hole.update", {{"hole", h["name"]}, {"depth", 10.0}});
        app.run("solve", {});
        app.run("build", {});
        app.display = modelapp::Display::deviation;
    }

    id<MTLTexture> depth_tex = nil, offscreen = nil;
    MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor new];
    pass.colorAttachments[0].clearColor = MTLClearColorMake(0.11, 0.12, 0.14, 1.0);
    pass.colorAttachments[0].loadAction = MTLLoadActionClear;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    pass.depthAttachment.loadAction = MTLLoadActionClear;
    pass.depthAttachment.storeAction = MTLStoreActionDontCare;
    pass.depthAttachment.clearDepth = 1.0;
    struct {
        bool rotating = false, panning = false;
        float last_x = 0, last_y = 0;
        float press_x = 0, press_y = 0;
    } mouse;
    const recon::TriangleMesh* framed_scan = nullptr;  // the scan the view was last fitted to
    auto next_frame = std::chrono::steady_clock::now();
    int snapshot_frames = 0;

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

            app.update();
            if (!app.busy() && app.doc.has_scan() && framed_scan != &app.doc.mesh()) {
                // A new scan: fit it in the view, looked at from the side the scanner saw (the demo part, whose
                // axes are the part's, from above and to the side).
                framed_scan = &app.doc.mesh();
                app.look_from(camera, app.doc.source().kind == "demo" ? "iso" : "scanned");
                app.frame(camera);
            }
            if (auto upd = app.take_render_update()) {
                if (upd->geometry) (*renderer)->set_mesh(upd->geometry->first, upd->geometry->second);
                (*renderer)->set_mesh_colors(upd->colors);
                (*renderer)->set_lines(upd->lines);
            }

            ImGuiIO& io = ImGui::GetIO();
            const bool mouse_valid = ImGui::IsMousePosValid(&io.MousePos);
            const float mx = mouse_valid ? io.MousePos.x : mouse.last_x, my = mouse_valid ? io.MousePos.y : mouse.last_y;
            const auto [ray_origin, ray_dir] = ray_at(camera, mx, my, io.DisplaySize.x, io.DisplaySize.y);
            // Brush: Shift-drag paints the selected label, Option-drag erases; one stroke is one undo step.
            const bool over_view = !io.WantCaptureMouse;
            if (app.stroking()) {
                if (io.MouseDown[0]) app.dab(ray_origin, ray_dir);
                else app.end_stroke();
            } else if (over_view && io.MouseClicked[0] && (io.KeyShift || io.KeyAlt) && app.doc.has_scan() && !app.busy()) {
                app.begin_stroke(io.KeyAlt && !io.KeyShift);
                app.dab(ray_origin, ray_dir);
            }
            // Dropped files: photos join the library, a scan or model is opened.
            if (!g_dropped.empty() && !app.busy()) {
                std::vector<std::filesystem::path> images;
                for (const auto& f : g_dropped)
                    if (is_photo(f)) images.push_back(f);
                    else app.open(f);
                photos.import_files(app, images);
                g_dropped.clear();
            }
            if (!io.WantTextInput && !app.busy()) {
                if (io.KeySuper && ImGui::IsKeyPressed(ImGuiKey_V, false)) photos.paste(app);
                if (io.KeySuper && ImGui::IsKeyPressed(ImGuiKey_Z, false)) app.run(io.KeyShift ? "redo" : "undo", {});
                if (ImGui::IsKeyPressed(ImGuiKey_F, false) && !io.KeySuper) app.frame(camera);
            }
            // Matching a photo to the scan: a click (not a drag) on the scan gives the point.
            if (over_view && photos.awaiting_scan_point() && !app.busy()) {
                if (io.MouseClicked[0]) mouse.press_x = mx, mouse.press_y = my;
                if (io.MouseReleased[0] && std::hypot(mx - mouse.press_x, my - mouse.press_y) < 4)
                    if (const auto hit = app.doc.bvh().raycast(ray_origin, ray_dir)) photos.scan_point(app, hit->point.cast<double>());
            }
            if (over_view && !app.stroking()) {
                const bool l = io.MouseDown[0], r = io.MouseDown[1] || io.MouseDown[2];
                const bool moved = mx != mouse.last_x || my != mouse.last_y;
                if (l && mouse.rotating && moved) camera.orbit((mx - mouse.last_x) * 0.008f, (my - mouse.last_y) * 0.008f);
                if (r && mouse.panning && moved) camera.pan(mx - mouse.last_x, my - mouse.last_y, io.DisplaySize.y);
                mouse.rotating = l && !io.KeyShift && !io.KeyAlt;
                mouse.panning = r;
                if (io.MouseWheel != 0.0f) camera.zoom(std::pow(0.9f, io.MouseWheel));
            } else {
                mouse.rotating = mouse.panning = false;
            }
            mouse.last_x = mx;
            mouse.last_y = my;

            id<CAMetalDrawable> drawable = nil;
            if (offscreen_mode) {
                if (!offscreen || int(offscreen.width) != fb_w || int(offscreen.height) != fb_h) {
                    MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                                                                width:NSUInteger(fb_w)
                                                                                               height:NSUInteger(fb_h)
                                                                                            mipmapped:NO];
                    d.usage = MTLTextureUsageRenderTarget;
                    d.storageMode = MTLStorageModePrivate;
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
            (*renderer)->encode((__bridge MTL::RenderCommandEncoder*)enc, camera, float(fb_w), float(fb_h), rs);

            ImGui_ImplMetal_NewFrame(pass);
            ImGui_ImplGlfw_NewFrame();
            if (agent) agent->pump();
            ImGui::NewFrame();
            app.draw_ui(camera);
            photos.draw(app, camera);

            // The brush outline and the label under the cursor.
            if (over_view && app.doc.has_scan() && !app.busy()) {
                if (const auto hit = app.doc.bvh().raycast(ray_origin, ray_dir)) {
                    const float dist = (hit->point - camera.eye()).norm();
                    const float px = app.brush_radius * (io.DisplaySize.y / 2) / (dist * std::tan(camera.fov_y / 2));
                    if (io.KeyShift || io.KeyAlt)
                        ImGui::GetForegroundDrawList()->AddCircle(ImVec2(mx, my), px, io.KeyAlt && !io.KeyShift ? IM_COL32(110, 190, 255, 220)
                                                                                                                  : IM_COL32(255, 140, 70, 220),
                                                                  0, 1.5f);
                    else if (const auto name = app.label_at(ray_origin, ray_dir); name && !io.MouseDown[0])
                        ImGui::SetTooltip("%s", name->c_str());
                }
            }
            // Status line.
            ImGui::SetNextWindowPos(ImVec2(0, io.DisplaySize.y - 26), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, 26), ImGuiCond_Always);
            ImGui::Begin("##status", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
            if (app.busy()) ImGui::TextColored(ImVec4(1, 0.85f, 0.3f, 1), "%s", app.busy_text().c_str());
            else ImGui::TextColored(app.status_is_error() ? ImVec4(1, 0.45f, 0.4f, 1) : ImVec4(0.7f, 0.7f, 0.7f, 1), "%s", app.status().c_str());
            ImGui::End();

            ImGui::Render();
            ImGui_ImplMetal_RenderDrawData(ImGui::GetDrawData(), cmd, enc);
            [enc endEncoding];
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
            if (snapshot_path && ++snapshot_frames == 3) {
                [cmd waitUntilCompleted];
                std::println("snapshot: {}", write_png(queue, offscreen, snapshot_path) ? snapshot_path : "FAILED");
                std::println("status: {}", app.status());
                break;
            }
            if (offscreen_mode) {
                std::this_thread::sleep_until(next_frame);
                next_frame = std::max(next_frame + std::chrono::microseconds(16667), std::chrono::steady_clock::now());
            }
        }
    }

    ImGui_ImplMetal_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
