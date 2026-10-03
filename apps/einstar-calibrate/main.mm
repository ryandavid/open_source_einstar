// einstar-calibrate: guided stereo calibration of the Einstar's IR pair from views of its calibration
// board, and a comparison of the result with the calibration EXStar stored in the scanner.
//
// GLFW window + CAMetalLayer + Dear ImGui. The live left camera view shows the detected board, where
// it is (solid outline) and where the next capture wants it (dashed); the side panel shows the
// capture plan as five distance ladders (one per board orientation), tilt and distance gauges and the
// solve. Captures happen automatically once the board is held still in the asked-for pose (or with the
// scanner's start / pause button).

#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#import <AppKit/AppKit.h>
#import <CoreGraphics/CoreGraphics.h>
#import <ImageIO/ImageIO.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <format>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
#include <vector>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_metal.h"

#include "agent_methods.hpp"
#include "board_view.hpp"
#include "einstar/agent/server.hpp"
#include "calibration_controller.hpp"
#include "einstar/core/timing.hpp"
#include "upright_image.hpp"

using namespace einstar;
using calibrate::BoardSpec;
using app::ImageFrame;

namespace {

struct GreyTexture {
    id<MTLTexture> texture = nil;
    int width = 0, height = 0;
    const ImageU8* uploaded = nullptr;

    void upload(id<MTLDevice> device, const std::shared_ptr<const ImageU8>& img) {
        if (!img || img->empty() || img.get() == uploaded) return;
        if (!texture || width != img->width() || height != img->height()) {
            MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm
                                                                                        width:NSUInteger(img->width())
                                                                                       height:NSUInteger(img->height())
                                                                                    mipmapped:NO];
            d.swizzle = MTLTextureSwizzleChannelsMake(MTLTextureSwizzleRed, MTLTextureSwizzleRed, MTLTextureSwizzleRed, MTLTextureSwizzleOne);
            texture = [device newTextureWithDescriptor:d];
            width = img->width();
            height = img->height();
        }
        [texture replaceRegion:MTLRegionMake2D(0, 0, NSUInteger(width), NSUInteger(height)) mipmapLevel:0 withBytes:img->data() bytesPerRow:NSUInteger(width)];
        uploaded = img.get();
    }
};

constexpr ImU32 kGreen = IM_COL32(60, 220, 100, 255);
constexpr ImU32 kAmber = IM_COL32(250, 180, 40, 255);
constexpr ImU32 kCyan = IM_COL32(70, 200, 255, 255);
constexpr ImU32 kGrey = IM_COL32(90, 90, 96, 255);
constexpr ImU32 kGroupColours[5] = {IM_COL32(90, 200, 255, 255), IM_COL32(255, 140, 90, 255), IM_COL32(200, 120, 255, 255),
                                    IM_COL32(120, 230, 120, 255), IM_COL32(255, 220, 80, 255)};
const char* kGroupShort[5] = {"Face-on", "Top near", "Bottom near", "Right near", "Left near"};

void dashed_line(ImDrawList* dl, ImVec2 a, ImVec2 b, ImU32 col, float thickness, float dash = 10) {
    const float dx = b.x - a.x, dy = b.y - a.y, len = std::sqrt(dx * dx + dy * dy);
    if (len <= 0) return;
    for (float s = 0; s < len; s += 2 * dash) {
        const float e = std::min(len, s + dash);
        dl->AddLine(ImVec2(a.x + dx * s / len, a.y + dy * s / len), ImVec2(a.x + dx * e / len, a.y + dy * e / len), col, thickness);
    }
}

// The board outline through a camera (dashed or solid). Returns false if it is behind the camera.
bool draw_outline(ImDrawList* dl, const ImageFrame& f, const CameraModel& cam, const SE3& T_cam_board, ImU32 col, bool dashed, float thickness) {
    const BoardSpec board;
    const auto o = board.outline();
    std::array<ImVec2, 4> p;
    for (std::size_t i = 0; i < 4; ++i) {
        // Subdivide the edges so lens distortion bends them like the real board edge.
        const Vec3 c = T_cam_board * o[i];
        if (c.z() <= 10) return false;
        p[i] = f.at(cam.project(c));
    }
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec3 a = o[i], b = o[(i + 1) % 4];
        ImVec2 prev = p[i];
        for (int k = 1; k <= 8; ++k) {
            const ImVec2 q = f.at(cam.project(T_cam_board * (a + (b - a) * (k / 8.0))));
            if (dashed) dashed_line(dl, prev, q, col, thickness, 7);
            else dl->AddLine(prev, q, col, thickness);
            prev = q;
        }
    }
    return true;
}

void draw_detection(ImDrawList* dl, const ImageFrame& f, const calibrate::BoardDetection& d) {
    for (const auto& px : d.pixels) dl->AddCircle(f.at(px), 4.0f, kGreen, 10, 1.5f);
    for (const auto& px : d.large) dl->AddCircle(f.at(px), 7.0f, kAmber, 12, 2.0f);
}

// Tilt target: board normal as a point in the view's terms (right edge near to the right: +tilt_x; top edge
// near up: +tilt_y), +-45 deg.
void draw_tilt_gauge(const calibrate::PoseTarget* t, const calibrate::BoardMeasure* m, float size) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 c(p0.x + size * 0.5f, p0.y + size * 0.5f);
    const float k = size * 0.5f / 45.0f;
    dl->AddRectFilled(p0, ImVec2(p0.x + size, p0.y + size), IM_COL32(30, 32, 36, 255), 6);
    dl->AddLine(ImVec2(p0.x + 4, c.y), ImVec2(p0.x + size - 4, c.y), kGrey);
    dl->AddLine(ImVec2(c.x, p0.y + 4), ImVec2(c.x, p0.y + size - 4), kGrey);
    for (const float r : {15.0f, 30.0f}) dl->AddCircle(c, r * k, IM_COL32(60, 60, 66, 255), 48);
    if (t) {
        const ImVec2 tc(c.x + static_cast<float>(t->tilt_x_deg) * k, c.y - static_cast<float>(t->tilt_y_deg) * k);
        dl->AddCircleFilled(tc, static_cast<float>(t->tilt_tol_deg) * k, IM_COL32(70, 200, 255, 60), 32);
        dl->AddCircle(tc, static_cast<float>(t->tilt_tol_deg) * k, kCyan, 32, 1.5f);
    }
    if (m) {
        const float x = std::clamp(static_cast<float>(m->tilt_x_deg), -45.f, 45.f), y = std::clamp(static_cast<float>(m->tilt_y_deg), -45.f, 45.f);
        const bool ok = t && std::hypot(m->tilt_y_deg - t->tilt_y_deg, m->tilt_x_deg - t->tilt_x_deg) <= t->tilt_tol_deg;
        dl->AddCircleFilled(ImVec2(c.x + x * k, c.y - y * k), 6, ok ? kGreen : kAmber, 16);
    }
    dl->AddText(ImVec2(p0.x + 4, p0.y + 2), IM_COL32(150, 150, 150, 255), "top near");
    dl->AddText(ImVec2(p0.x + 4, p0.y + size - 16), IM_COL32(150, 150, 150, 255), "bottom near");
    const char* r = "right near";
    dl->AddText(ImVec2(p0.x + size - ImGui::CalcTextSize(r).x - 4, c.y + 2), IM_COL32(150, 150, 150, 255), r);
    dl->AddText(ImVec2(p0.x + 4, c.y + 2), IM_COL32(150, 150, 150, 255), "left near");
    ImGui::Dummy(ImVec2(size, size));
}

// Distance: 150..650 mm, target band and the current distance.
void draw_distance_gauge(const calibrate::PoseTarget* t, const calibrate::BoardMeasure* m, float width) {
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const float h = 22, lo = 150, hi = 650;
    auto x_of = [&](double mm) { return p0.x + width * std::clamp(static_cast<float>((mm - lo) / (hi - lo)), 0.f, 1.f); };
    dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + h), IM_COL32(30, 32, 36, 255), 4);
    for (int mm = 200; mm <= 600; mm += 100) {
        dl->AddLine(ImVec2(x_of(mm), p0.y + h - 6), ImVec2(x_of(mm), p0.y + h), kGrey);
        const auto s = std::format("{}", mm);
        dl->AddText(ImVec2(x_of(mm) - ImGui::CalcTextSize(s.c_str()).x * 0.5f, p0.y + h + 1), IM_COL32(130, 130, 130, 255), s.c_str());
    }
    if (t) dl->AddRectFilled(ImVec2(x_of(t->distance_mm - t->distance_tol_mm), p0.y + 3), ImVec2(x_of(t->distance_mm + t->distance_tol_mm), p0.y + h - 3), IM_COL32(70, 200, 255, 110), 3);
    if (m) {
        const bool ok = t && std::abs(m->distance_mm - t->distance_mm) <= t->distance_tol_mm;
        const float x = x_of(m->distance_mm);
        dl->AddTriangleFilled(ImVec2(x - 6, p0.y - 2), ImVec2(x + 6, p0.y - 2), ImVec2(x, p0.y + 8), ok ? kGreen : kAmber);
        dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p0.y + h), ok ? kGreen : kAmber, 2);
    }
    ImGui::Dummy(ImVec2(width, h + 16));
}

// The plan: one ladder per orientation, nearest step at the bottom. Returns a clicked group or -1.
int draw_ladders(const app::CalibrationController& c, const std::vector<std::optional<app::CaptureRecord>>& caps, const app::LiveState& live, float width) {
    const auto& plan = c.plan();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const float colw = width / 5.0f, boxw = colw - 14, boxh = 20, gap = 5, top = 4;
    const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(ImGui::GetTime()) * 6.0f);
    int clicked = -1;
    for (int g = 0; g < 5; ++g) {
        const float x0 = p0.x + static_cast<float>(g) * colw + 7;
        const bool active = g == c.active_group();
        if (active) dl->AddRectFilled(ImVec2(x0 - 5, p0.y), ImVec2(x0 + boxw + 5, p0.y + top + 5 * (boxh + gap) + 34), IM_COL32(50, 60, 75, 255), 6);
        for (int s = 0; s < 5; ++s) {
            const int idx = g * 5 + s;
            const float y = p0.y + top + static_cast<float>(4 - s) * (boxh + gap);
            const ImVec2 a(x0, y), b(x0 + boxw, y + boxh);
            if (caps[static_cast<std::size_t>(idx)]) {
                dl->AddRectFilled(a, b, IM_COL32(40, 150, 70, 255), 3);
                dl->AddText(ImVec2(x0 + boxw * 0.5f - 4, y + 3), IM_COL32(230, 255, 230, 255), "v");
            } else {
                dl->AddRectFilled(a, b, IM_COL32(28, 30, 34, 255), 3);
                dl->AddRect(a, b, IM_COL32(70, 110, 160, 255), 3);
            }
            if (idx == live.target) dl->AddRect(ImVec2(a.x - 2, a.y - 2), ImVec2(b.x + 2, b.y + 2), IM_COL32(70, 200, 255, static_cast<int>(120 + 135 * pulse)), 4, 0, 2.5f);
            const auto label = std::format("{:.0f}", plan[static_cast<std::size_t>(idx)].distance_mm);
            if (!caps[static_cast<std::size_t>(idx)]) dl->AddText(ImVec2(x0 + boxw * 0.5f - ImGui::CalcTextSize(label.c_str()).x * 0.5f, y + 3), IM_COL32(120, 140, 170, 255), label.c_str());
        }
        // Where the scanner is now, on the active ladder.
        if (active && live.measure) {
            const double d = live.measure->distance_mm;
            const auto& p = plan;
            const double d0 = p[static_cast<std::size_t>(g * 5)].distance_mm, d4 = p[static_cast<std::size_t>(g * 5 + 4)].distance_mm;
            const float frac = static_cast<float>((d - d0) / (d4 - d0));
            const float y = p0.y + top + (4 - std::clamp(frac * 4.0f, -0.6f, 4.6f)) * (boxh + gap) + boxh * 0.5f;
            dl->AddTriangleFilled(ImVec2(x0 - 5, y - 5), ImVec2(x0 - 5, y + 5), ImVec2(x0 + 2, y), IM_COL32(255, 255, 255, 230));
            dl->AddLine(ImVec2(x0, y), ImVec2(x0 + boxw, y), IM_COL32(255, 255, 255, 160), 1.5f);
        }
        const char* name = kGroupShort[g];
        const float tw = ImGui::CalcTextSize(name).x;
        dl->AddText(ImVec2(x0 + boxw * 0.5f - tw * 0.5f, p0.y + top + 5 * (boxh + gap) + 2), active ? IM_COL32(230, 230, 230, 255) : IM_COL32(140, 140, 140, 255), name);
        // Small orientation glyph: which edge is near.
        const ImVec2 gc(x0 + boxw * 0.5f, p0.y + top + 5 * (boxh + gap) + 24);
        const float r = 7;
        const ImVec2 q[4] = {{gc.x - r, gc.y - r * 0.6f}, {gc.x + r, gc.y - r * 0.6f}, {gc.x + r, gc.y + r * 0.6f}, {gc.x - r, gc.y + r * 0.6f}};
        ImVec2 w[4] = {q[0], q[1], q[2], q[3]};
        const float e = 2.5f;
        if (g == 1) w[0].x -= e, w[1].x += e;  // top edge near: drawn larger
        if (g == 2) w[2].x += e, w[3].x -= e;
        if (g == 3) w[1].y -= e, w[2].y += e;
        if (g == 4) w[0].y -= e, w[3].y += e;
        dl->AddQuad(w[0], w[1], w[2], w[3], kGroupColours[g], 1.5f);
        ImGui::SetCursorScreenPos(ImVec2(x0 - 5, p0.y));
        ImGui::InvisibleButton(std::format("##group{}", g).c_str(), ImVec2(boxw + 10, top + 5 * (boxh + gap) + 34));
        if (ImGui::IsItemClicked()) clicked = g;
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s: click to capture this orientation next", kGroupShort[g]);
    }
    ImGui::SetCursorScreenPos(ImVec2(p0.x, p0.y + top + 5 * (boxh + gap) + 40));
    ImGui::Dummy(ImVec2(width, 1));
    return clicked;
}

std::string choose_directory() {
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseDirectories = YES;
    panel.canChooseFiles = NO;
    panel.allowsMultipleSelection = NO;
    panel.message = @"Choose a folder of calibration captures (imageLeftN / imageRightN)";
    if ([panel runModal] == NSModalResponseOK) return panel.URL.path.UTF8String;
    return {};
}

std::string choose_path() {
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseDirectories = YES;
    panel.canChooseFiles = YES;
    panel.allowsMultipleSelection = NO;
    panel.message = @"Choose a calibration: EXStar's CCF folder, a flash dump (.bin) or an einstar-calibrate calibration.txt";
    if ([panel runModal] == NSModalResponseOK) return panel.URL.path.UTF8String;
    return {};
}

std::string choose_file() {
    NSOpenPanel* panel = [NSOpenPanel openPanel];
    panel.canChooseDirectories = NO;
    panel.canChooseFiles = YES;
    panel.allowsMultipleSelection = NO;
    panel.message = @"Choose a calibration backup (flash-backup-*.bin)";
    if ([panel runModal] == NSModalResponseOK) return panel.URL.path.UTF8String;
    return {};
}

// Writes a BGRA8 texture to PNG (used by --snapshot). The texture is GPU-private; a blit on `queue`, after the
// frames already queued, copies it into a shared buffer the CPU can read on any GPU.
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

std::string fmt_signed(double v, int prec) { return std::format("{:+.{}f}", v, prec); }

void results_tab(app::CalibrationController& c, const app::SolveState& st) {
    if (st.running) {
        ImGui::TextUnformatted("Solving...");
        return;
    }
    if (!st.error.empty()) {
        ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "Solve failed: %s", st.error.c_str());
        return;
    }
    if (!st.ours) {
        ImGui::TextWrapped("Capture the views (or load a folder of captures), then Solve. The result is compared here with the calibration stored in "
                           "the scanner, which EXStar made, evaluated on the same captures.");
        return;
    }
    const auto& o = *st.ours;
    ImGui::Text("Ours:  reprojection %.3f px, rectified rows %.3f px (max %.2f), %d dots in %d views, %d outliers dropped, %.0f ms", o.rms_px, o.row_rms_px,
                o.max_row_px, o.dots, static_cast<int>(std::ranges::count_if(o.views, &calibrate::ViewReport::used)), o.dropped, st.solve_ms);
    if (st.flash)
        ImGui::Text("Flash (EXStar, %s) on these captures:  reprojection %.3f px, rectified rows %.3f px (max %.2f)", c.flash_time().c_str(), st.flash->rms_px,
                    st.flash->row_rms_px, st.flash->max_row_px);
    if (st.factory)
        ImGui::Text("Factory section on these captures:  reprojection %.3f px, rectified rows %.3f px (max %.2f)", st.factory->rms_px, st.factory->row_rms_px,
                    st.factory->max_row_px);
    ImGui::TextDisabled("Rectified rows: how far apart a dot lands in the two rectified images; stereo matching needs this near 0.");
    ImGui::Separator();

    const auto* flash = c.flash_rig() ? &*c.flash_rig() : nullptr;
    const auto* factory = c.factory_rig() ? &*c.factory_rig() : nullptr;
    const int cols = 2 + (flash ? 2 : 0) + (factory ? 2 : 0);
    if (ImGui::BeginTable("params", cols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("Parameter");
        ImGui::TableSetupColumn("Ours");
        if (flash) ImGui::TableSetupColumn("Flash (EXStar)"), ImGui::TableSetupColumn("Ours - flash");
        if (factory) ImGui::TableSetupColumn("Factory"), ImGui::TableSetupColumn("Ours - factory");
        ImGui::TableHeadersRow();
        auto row = [&](const std::string& name, auto get, int prec) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(name.c_str());
            const double v = get(o.rig);
            ImGui::TableNextColumn();
            ImGui::Text("%.*f", prec, v);
            for (const auto* ref : {flash, factory}) {
                if (!ref) continue;
                const double r = get(*ref);
                ImGui::TableNextColumn();
                ImGui::Text("%.*f", prec, r);
                ImGui::TableNextColumn();
                const double d = v - r;
                ImGui::TextColored(std::abs(d) > std::pow(10.0, -prec + 1) * 5 ? ImVec4(1, 0.75f, 0.3f, 1) : ImVec4(0.7f, 0.7f, 0.7f, 1), "%s", fmt_signed(d, prec).c_str());
            }
        };
        for (int cam = 0; cam < 2; ++cam) {
            const char* n = cam == 0 ? "Left" : "Right";
            auto m = [cam](const RigCalibration& r) -> const CameraModel& { return cam == 0 ? r.left : r.right; };
            row(std::format("{} fx", n), [&](const RigCalibration& r) { return m(r).fx; }, 2);
            row(std::format("{} fy", n), [&](const RigCalibration& r) { return m(r).fy; }, 2);
            row(std::format("{} cx", n), [&](const RigCalibration& r) { return m(r).cx; }, 2);
            row(std::format("{} cy", n), [&](const RigCalibration& r) { return m(r).cy; }, 2);
            const char* kn[5] = {"k1", "k2", "p1", "p2", "k3"};
            for (int k = 0; k < 5; ++k) row(std::format("{} {}", n, kn[k]), [&, k](const RigCalibration& r) { return m(r).dist[static_cast<std::size_t>(k)]; }, 5);
        }
        auto rot = [](const RigCalibration& r, int i) {
            const Eigen::AngleAxisd aa(r.T_right_left.linear());
            return (aa.axis() * aa.angle() * 180.0 / M_PI)(i);
        };
        row("Rig rotation x (deg)", [&](const RigCalibration& r) { return rot(r, 0); }, 4);
        row("Rig rotation y (deg)", [&](const RigCalibration& r) { return rot(r, 1); }, 4);
        row("Rig rotation z (deg)", [&](const RigCalibration& r) { return rot(r, 2); }, 4);
        row("Rig t x (mm)", [](const RigCalibration& r) { return r.T_right_left.translation().x(); }, 3);
        row("Rig t y (mm)", [](const RigCalibration& r) { return r.T_right_left.translation().y(); }, 3);
        row("Rig t z (mm)", [](const RigCalibration& r) { return r.T_right_left.translation().z(); }, 3);
        row("Baseline (mm)", [](const RigCalibration& r) { return r.baseline_mm(); }, 3);
        ImGui::EndTable();
    }
    for (const auto& [name, d] : {std::pair{"flash", st.vs_flash}, std::pair{"factory", st.vs_factory}}) {
        if (!d) continue;
        ImGui::Text("vs %s: rig turned %.3f deg; a ray lands up to %.2f px (left) / %.2f px (right) apart over the image, distortion alone %.2f / %.2f px", name,
                    d->rotation_deg.norm(), d->left.mapping_px, d->right.mapping_px, d->left.distortion_px, d->right.distortion_px);
    }
    ImGui::Separator();
    // Per-view rows: ours vs flash.
    std::vector<float> ours_rows, flash_rows;
    for (std::size_t i = 0; i < o.views.size(); ++i) {
        ours_rows.push_back(static_cast<float>(o.views[i].row_rms_px));
        if (st.flash && i < st.flash->views.size()) flash_rows.push_back(static_cast<float>(st.flash->views[i].row_rms_px));
    }
    const float hmax = std::max(0.2f, std::max(ours_rows.empty() ? 0.f : *std::ranges::max_element(ours_rows), flash_rows.empty() ? 0.f : *std::ranges::max_element(flash_rows)));
    ImGui::PlotHistogram("##ours_rows", ours_rows.data(), static_cast<int>(ours_rows.size()), 0, "rectified row RMS per view: ours", 0, hmax, ImVec2(420, 70));
    if (!flash_rows.empty()) {
        ImGui::SameLine();
        ImGui::PlotHistogram("##flash_rows", flash_rows.data(), static_cast<int>(flash_rows.size()), 0, "flash", 0, hmax, ImVec2(420, 70));
    }
    if (ImGui::BeginTable("views", st.flash ? 8 : 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollY,
                          ImVec2(0, 0))) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("View");
        ImGui::TableSetupColumn("Dots");
        ImGui::TableSetupColumn("Distance");
        ImGui::TableSetupColumn("Tilt x / y");
        ImGui::TableSetupColumn("Ours rms");
        ImGui::TableSetupColumn("Ours rows");
        if (st.flash) ImGui::TableSetupColumn("Flash rms"), ImGui::TableSetupColumn("Flash rows");
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < o.views.size(); ++i) {
            const auto& v = o.views[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(v.name.c_str());
            if (!v.used) {
                ImGui::TableNextColumn();
                ImGui::TextDisabled("not used");
                continue;
            }
            ImGui::TableNextColumn();
            ImGui::Text("%d", v.dots);
            ImGui::TableNextColumn();
            ImGui::Text("%.0f mm", v.distance_mm);
            ImGui::TableNextColumn();
            ImGui::Text("%+.0f / %+.0f", v.tilt_x_deg, v.tilt_y_deg);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", v.rms_px);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", v.row_rms_px);
            if (st.flash) {
                ImGui::TableNextColumn();
                ImGui::Text("%.3f", st.flash->views[i].rms_px);
                ImGui::TableNextColumn();
                const double fr = st.flash->views[i].row_rms_px;
                ImGui::TextColored(fr > 0.3 ? ImVec4(1, 0.5f, 0.3f, 1) : ImVec4(0.8f, 0.8f, 0.8f, 1), "%.3f", fr);
            }
        }
        ImGui::EndTable();
    }
}

// Every captured dot over the two image frames: where the calibration has data.
void coverage_tab(const app::CalibrationController& c, const std::vector<std::optional<app::CaptureRecord>>& caps, const app::LiveState& live) {
    // Each camera's image as the view shows it (upright: 1024 wide, 1280 tall).
    const float avail = ImGui::GetContentRegionAvail().x;
    const float h = (avail - 20) * 0.5f * 1024.0f / 1280.0f;
    const float k = h / 1280.0f, w = 1024.0f * k;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    for (int cam = 0; cam < 2; ++cam) {
        if (cam == 1) ImGui::SameLine(0, 20);
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const ImageFrame f{p0, k, 1280.0f};
        dl->AddRectFilled(p0, ImVec2(p0.x + w, p0.y + h), IM_COL32(22, 23, 26, 255));
        dl->AddRect(p0, ImVec2(p0.x + w, p0.y + h), kGrey);
        // Coverage grid: 8 x 6 cells, shaded by the number of dots seen there.
        int cells[6][8] = {};
        for (const auto& r : caps)
            if (r)
                for (const auto& px : (cam == 0 ? r->capture.left : r->capture.right).pixels) {
                    const int cx = std::clamp(static_cast<int>(px.x() / 160.0), 0, 7), cy = std::clamp(static_cast<int>(px.y() / (1024.0 / 6)), 0, 5);
                    ++cells[cy][cx];
                }
        for (int y = 0; y < 6; ++y)
            for (int x = 0; x < 8; ++x) {
                const int n = cells[y][x];
                const ImU32 col = n == 0 ? IM_COL32(120, 40, 40, 70) : n < 8 ? IM_COL32(160, 130, 40, 70) : IM_COL32(40, 140, 70, 60);
                // The cell's image corners, drawn turned: min / max of the two on screen.
                const ImVec2 a = f.at(Vec2(160.0 * x, 1024.0 / 6 * y)), b = f.at(Vec2(160.0 * (x + 1), 1024.0 / 6 * (y + 1)));
                dl->AddRectFilled(ImVec2(std::min(a.x, b.x) + 1, std::min(a.y, b.y) + 1), ImVec2(std::max(a.x, b.x) - 1, std::max(a.y, b.y) - 1), col);
            }
        for (std::size_t i = 0; i < caps.size(); ++i)
            if (caps[i])
                for (const auto& px : (cam == 0 ? caps[i]->capture.left : caps[i]->capture.right).pixels)
                    dl->AddCircleFilled(f.at(px), 2.0f, kGroupColours[c.plan()[i].group], 6);
        const auto& d = cam == 0 ? live.det_left : live.det_right;
        if (d)
            for (const auto& px : d->pixels) dl->AddCircle(f.at(px), 4.0f, IM_COL32(255, 255, 255, 200), 8);
        dl->AddText(ImVec2(p0.x + 6, p0.y + 4), IM_COL32(200, 200, 200, 255), cam == 0 ? "Left camera" : "Right camera");
        ImGui::Dummy(ImVec2(w, h));
    }
    ImGui::TextDisabled("Dots of every capture, coloured by orientation (white: live). Red cells have no data: distortion there is extrapolated.");
    if (ImGui::BeginTable("caps", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
        for (const char* h2 : {"#", "Target", "Captured", "Dots L / R", "Distance", "Tilt x / y", ""}) ImGui::TableSetupColumn(h2);
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < caps.size(); ++i) {
            if (!caps[i]) continue;
            const auto& r = *caps[i];
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%zu", i + 1);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(c.plan()[i].label.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(r.time.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%zu / %zu", r.capture.left.size(), r.capture.right.size());
            ImGui::TableNextColumn();
            ImGui::Text("%.0f mm", r.measure.distance_mm);
            ImGui::TableNextColumn();
            ImGui::Text("%+.0f / %+.0f", r.measure.tilt_x_deg, r.measure.tilt_y_deg);
            ImGui::TableNextColumn();
            if (ImGui::SmallButton(std::format("Retake##{}", i).c_str())) const_cast<app::CalibrationController&>(c).clear_capture(static_cast<int>(i));
        }
        ImGui::EndTable();
    }
}

}  // namespace

int main(int argc, char** argv) {
    // --snapshot <out.png> [seconds]: headless emulator run, snapshot after `seconds`; with --complete, runs
    // the whole emulated procedure and the solve first and snapshots the results tab (--tab live|coverage
    // to snapshot another tab).
    const char* snapshot_path = nullptr;
    double snapshot_seconds = 6.0;
    bool snapshot_complete = false;
    bool snapshot_write = false;
    std::string snapshot_tab;
    std::string load_dir, reference_path;  // --load <captures dir> [--reference <calibration>]: offline
    // --mcp[=<socket>] [--visible]: agent control (libs/agent, einstar-mcp). The window is hidden unless
    // --visible; captures go to a temporary folder unless EINSTAR_CALIBRATION_DIR says otherwise.
    std::optional<std::string> mcp_socket;
    bool mcp_visible = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--mcp") mcp_socket = std::format("/tmp/einstar_calibration_{}.sock", ::getpid());
        if (a.starts_with("--mcp=")) mcp_socket = std::string(a.substr(6));
        if (a == "--visible") mcp_visible = true;
        if (a == "--snapshot" && i + 1 < argc) {
            snapshot_path = argv[i + 1];
            if (i + 2 < argc && argv[i + 2][0] != '-') snapshot_seconds = std::atof(argv[i + 2]);
        }
        if (a == "--complete") snapshot_complete = true;
        if (a == "--write") snapshot_write = true;  // with --complete: write the result into the (emulated) scanner
        if (a == "--tab" && i + 1 < argc) snapshot_tab = argv[i + 1];
        if (a == "--load" && i + 1 < argc) load_dir = argv[i + 1];
        if (a == "--reference" && i + 1 < argc) reference_path = argv[i + 1];
    }
    if (snapshot_path) {
        // Headless runs keep their captures in a temporary directory, never the user's.
        const auto tmp = std::filesystem::temp_directory_path() / "einstar_calibrate_snapshot";
        setenv("EINSTAR_CALIBRATION_DIR", tmp.c_str(), 1);
    }
    if (!glfwInit()) {
        std::println(stderr, "glfwInit failed");
        return 1;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    // Headless (a snapshot, or an agent without --visible): rendered offscreen, paced like a display.
    const bool offscreen_mode = snapshot_path || (mcp_socket && !mcp_visible);
    if (offscreen_mode) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    if (mcp_socket && !std::getenv("EINSTAR_CALIBRATION_DIR"))
        setenv("EINSTAR_CALIBRATION_DIR", (std::filesystem::temp_directory_path() / "einstar_agent_calibration").c_str(), 1);
    GLFWwindow* window = glfwCreateWindow(1560, 980, "Einstar Calibration", nullptr, nullptr);
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
    ImGui::StyleColorsDark();
    ImGui::GetStyle().WindowRounding = 0;
    ImGui_ImplGlfw_InitForOther(window, true);
    ImGui_ImplMetal_Init(device);

    app::CalibrationController ctl;
    GreyTexture tex_left, tex_right;
    std::string error;
    MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor new];
    pass.colorAttachments[0].clearColor = MTLClearColorMake(0.09, 0.10, 0.11, 1.0);
    pass.colorAttachments[0].loadAction = MTLLoadActionClear;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLTexture> offscreen = nil;
    Stopwatch clock;
    int snapshot_frames = -1;
    int tab_request = -1;  // 0 live, 1 coverage, 2 results
    if (!reference_path.empty())
        if (auto r = ctl.load_reference(reference_path); !r) error = r.error().message;
    if (!load_dir.empty()) {
        if (auto r = ctl.load_folder(load_dir); !r) error = r.error().message;
        else tab_request = 1;
    }
    if (snapshot_path && load_dir.empty()) {
        if (auto r = ctl.connect(true); !r) {
            std::println(stderr, "{}", r.error().message);
            return 1;
        }
        if (snapshot_tab == "coverage") tab_request = 1;
        if (snapshot_tab == "live") tab_request = 0;
    }
    bool solve_requested = false;
    app::CalibrationController::WritePlan write_plan;
    bool write_confirmed = false;
    std::string write_result, restore_path;
    app::BoardView board_view;
    std::unique_ptr<agent::Server> agent;
    if (mcp_socket) {
        agent = std::make_unique<agent::Server>(agent::App::calibration, "calibration");
        agent->set_visible(mcp_visible);
        app::register_calibration_agent(*agent, {ctl, tab_request, error});
        std::string why;
        if (!agent->start(*mcp_socket, why)) {
            std::println(stderr, "--mcp: {}", why);
            return 1;
        }
        std::println("AGENT_READY {} {}", *mcp_socket, ::getpid());
        std::fflush(stdout);
    }

    while (!glfwWindowShouldClose(window) && !(agent && agent->quit_requested())) {
        @autoreleasepool {
            glfwPollEvents();
            int fb_w = 0, fb_h = 0;
            glfwGetFramebufferSize(window, &fb_w, &fb_h);
            if (fb_w == 0 || fb_h == 0) continue;
            layer.drawableSize = CGSizeMake(fb_w, fb_h);

            const auto live = ctl.live();
            const auto caps = ctl.captures();
            const auto st = ctl.solve_state();
            const int captured = static_cast<int>(std::ranges::count_if(caps, [](const auto& c) { return c.has_value(); }));
            tex_left.upload(device, live.left);
            tex_right.upload(device, live.right);

            id<CAMetalDrawable> drawable = nil;
            if (offscreen_mode) {
                if (!offscreen || int(offscreen.width) != fb_w || int(offscreen.height) != fb_h) {
                    MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm width:NSUInteger(fb_w) height:NSUInteger(fb_h) mipmapped:NO];
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
            id<MTLCommandBuffer> cmd = [queue commandBuffer];
            id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:pass];
            ImGui_ImplMetal_NewFrame(pass);
            ImGui_ImplGlfw_NewFrame();
            if (agent) agent->pump();  // the agent's requests and synthetic input, before the frame
            ImGui::NewFrame();
            ImGuiIO& io = ImGui::GetIO();
            const float panel_w = 420;
            const auto* target = live.target >= 0 ? &ctl.plan()[static_cast<std::size_t>(live.target)] : nullptr;
            const auto* meas = live.measure ? &*live.measure : nullptr;
            const auto& grig = ctl.guidance_rig();

            // ---------------- main view ----------------
            ImGui::SetNextWindowPos(ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x - panel_w, io.DisplaySize.y));
            ImGui::Begin("##view", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus);
            if (ImGui::BeginTabBar("tabs")) {
                auto flags = [&](int t) { return tab_request == t ? ImGuiTabItemFlags_SetSelected : 0; };
                if (ImGui::BeginTabItem("Live", nullptr, flags(0))) {
                    // Instruction banner.
                    {
                        std::string text;
                        ImU32 col = kAmber;
                        if (!ctl.connected()) {
                            text = "Connect the scanner (or use the emulator) to start";
                            col = IM_COL32(180, 180, 180, 255);
                        } else if (!target) {
                            text = captured > 0 ? "All views captured: Solve" : "Nothing left to capture";
                            col = kGreen;
                        } else if (!live.det_left) {
                            text = "Point the scanner at the calibration board";
                        } else if (live.common_dots < 20) {
                            text = "Keep the whole board in view of both cameras";
                        } else if (live.guidance && !live.guidance->ok()) {
                            text = live.guidance->hints.front();
                        } else {
                            text = ctl.auto_capture ? "Hold still..." : "In position: press Capture";
                            col = kGreen;
                        }
                        ImGui::SetWindowFontScale(1.7f);
                        ImGui::PushStyleColor(ImGuiCol_Text, col);
                        ImGui::TextUnformatted(text.c_str());
                        ImGui::PopStyleColor();
                        ImGui::SetWindowFontScale(1.0f);
                        if (target) ImGui::Text("Next: %s   (%d of %zu captured)", target->label.c_str(), captured, ctl.plan().size());
                        if (live.guidance && live.guidance->hints.size() > 1)
                            for (std::size_t i = 1; i < live.guidance->hints.size(); ++i) {
                                ImGui::SameLine();
                                ImGui::TextDisabled("  |  %s", live.guidance->hints[i].c_str());
                            }
                    }
                    // The board-centred 3D guide, with the live left image as an inset.
                    const ImVec2 avail = ImGui::GetContentRegionAvail();
                    const ImVec2 view_o = ImGui::GetCursorScreenPos();
                    {
                        app::BoardViewInput bv;
                        bv.plan = &ctl.plan();
                        bv.captured.resize(caps.size());
                        bv.ghosts.resize(caps.size());
                        for (std::size_t i = 0; i < caps.size(); ++i)
                            if (caps[i]) {
                                bv.captured[i] = true;
                                bv.ghosts[i] = calibrate::board_from_scanner(caps[i]->measure);
                            }
                        bv.target = live.target;
                        if (meas) bv.scanner = calibrate::board_from_scanner(*meas);
                        bv.in_position = live.guidance && live.guidance->ok();
                        bv.aim_ok = live.guidance && live.guidance->offset_ok;
                        bv.steady_frac = live.steady_s / std::max(1e-3, live.steady_needed_s);
                        board_view.draw(bv, avail);
                    }
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    if (tex_left.texture) {
                        // Upright (the view): taller than wide.
                        const auto iw = static_cast<float>(tex_left.width), ih = static_cast<float>(tex_left.height);
                        const float s = std::min(std::min(380.0f, avail.x * 0.28f) / ih, avail.y * 0.55f / iw);
                        const float lw = ih * s, lh = iw * s;
                        const ImVec2 o(view_o.x + avail.x - lw - 12, view_o.y + avail.y - lh - 40);  // bottom right
                        const ImageFrame f{o, s, iw};
                        f.draw_image(dl, (ImTextureID)(__bridge void*)tex_left.texture, ih);
                        dl->AddRect(ImVec2(o.x - 1, o.y - 1), ImVec2(o.x + lw + 1, o.y + lh + 1), IM_COL32(120, 120, 128, 255));
                        dl->PushClipRect(o, ImVec2(o.x + lw, o.y + lh), true);
                        if (live.det_left) draw_detection(dl, f, *live.det_left);
                        if (target) draw_outline(dl, f, grig.left, calibrate::target_pose(*target, target->roll_deg, grig), kCyan, true, 2.0f);
                        if (live.pose) {
                            const bool ok = live.guidance && live.guidance->ok();
                            draw_outline(dl, f, grig.left, live.pose->T_cam_board, ok ? kGreen : kAmber, false, 2.0f);
                        }
                        dl->AddText(ImVec2(o.x + 6, o.y + 4), IM_COL32(230, 230, 230, 255), "Left camera");
                        dl->PopClipRect();
                        // Numbers under the inset.
                        std::vector<std::string> lines;
                        lines.push_back(std::format("Dots: left {}, right {}, both {}", live.det_left ? live.det_left->size() : 0,
                                                    live.det_right ? live.det_right->size() : 0, live.common_dots));
                        lines.push_back(std::format("Image level {}, saturated {:.1f}%   {:.1f} fps", live.mean_level, live.saturated_permille / 10.0, live.fps));
                        if (meas) {
                            lines.push_back(std::format("Distance {:.0f} mm   off centre {:.0f} mm", meas->distance_mm, meas->offset_mm.norm()));
                            // Roll as the view shows it: 0 with the board's long side across the view (the image's 90).
                            lines.push_back(std::format("Tilt x / y {:+.1f} / {:+.1f} deg   roll {:+.0f} deg", meas->tilt_x_deg, meas->tilt_y_deg,
                                                        std::remainder(meas->roll_deg - 90.0, 360.0)));
                        }
                        float y = o.y - 6 - static_cast<float>(lines.size()) * (ImGui::GetTextLineHeight() + 1);  // above the inset
                        for (const auto& l : lines) {
                            dl->AddText(ImVec2(o.x, y), IM_COL32(170, 170, 170, 255), l.c_str());
                            y += ImGui::GetTextLineHeight() + 1;
                        }
                    }
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem(std::format("Captures ({})###captures", captured).c_str(), nullptr, flags(1))) {
                    coverage_tab(ctl, caps, live);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Results", nullptr, flags(2))) {
                    results_tab(ctl, st);
                    ImGui::EndTabItem();
                }
                tab_request = -1;
                ImGui::EndTabBar();
            }
            ImGui::End();

            // ---------------- side panel ----------------
            ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - panel_w, 0));
            ImGui::SetNextWindowSize(ImVec2(panel_w, io.DisplaySize.y));
            ImGui::Begin("Calibration", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
            if (!ctl.connected()) {
                if (ImGui::Button("Connect scanner")) {
                    if (auto r = ctl.connect(false); !r) error = r.error().message;
                    else error.clear();
                }
                ImGui::SameLine();
                if (ImGui::Button("Use emulator")) {
                    if (auto r = ctl.connect(true); !r) error = r.error().message;
                    else error.clear();
                }
            } else {
                ImGui::TextWrapped("%s", ctl.description().c_str());
                if (ImGui::Button("Disconnect")) ctl.disconnect();
            }
            if (!error.empty()) ImGui::TextColored(ImVec4(1, 0.45f, 0.35f, 1), "%s", error.c_str());
            if (const auto s = ctl.status(); !s.empty()) ImGui::TextWrapped("%s", s.c_str());
            ImGui::Separator();

            ImGui::Text("Progress");
            ImGui::ProgressBar(static_cast<float>(captured) / static_cast<float>(ctl.plan().size()), ImVec2(-1, 0),
                               std::format("{} / {} views", captured, ctl.plan().size()).c_str());
            if (const int g = draw_ladders(ctl, caps, live, ImGui::GetContentRegionAvail().x); g >= 0) ctl.set_active_group(g);
            ImGui::TextDisabled("Each ladder: one board orientation, nearest distance at the bottom.");
            ImGui::Separator();

            ImGui::Text("Distance (mm)");
            draw_distance_gauge(target, meas, ImGui::GetContentRegionAvail().x);
            ImGui::Text("Board tilt");
            const float gauge = std::min(200.0f, ImGui::GetContentRegionAvail().x);
            draw_tilt_gauge(target, meas, gauge);
            ImGui::SameLine();
            ImGui::BeginGroup();
            ImGui::TextDisabled("cyan: target");
            ImGui::TextDisabled("dot: board now");
            if (target && meas) {
                ImGui::Text("x %+5.1f -> %+3.0f", meas->tilt_x_deg, target->tilt_x_deg);
                ImGui::Text("y %+5.1f -> %+3.0f", meas->tilt_y_deg, target->tilt_y_deg);
            }
            ImGui::EndGroup();
            {
                bool autoc = ctl.auto_capture;
                if (ImGui::Checkbox("Capture automatically when steady", &autoc)) ctl.auto_capture = autoc;
            }
            ImGui::BeginDisabled(!ctl.connected() || live.common_dots < 20 || !target);
            if (ImGui::Button("Capture now", ImVec2(-1, 0))) ctl.capture_now();
            ImGui::EndDisabled();
            ImGui::TextDisabled("(or the scanner's start / pause button)");
            ImGui::Separator();

            ImGui::BeginDisabled(captured < 4 || st.running);
            if (ImGui::Button(st.running ? "Solving..." : std::format("Solve ({} views)", captured).c_str(), ImVec2(-1, 0))) {
                ctl.solve();
                tab_request = 2;
            }
            ImGui::EndDisabled();
            ImGui::Checkbox("Keep the factory distortion", &ctl.keep_factory_distortion);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("EXStar's quick calibration re-fits focal length, principal point and the rig but keeps the factory "
                                                          "distortion. Off: everything is fitted from these captures.");
            if (st.ours) {
                ImGui::Text("Rows %.3f px, reprojection %.3f px", st.ours->row_rms_px, st.ours->rms_px);
                if (st.flash) ImGui::Text("(flash on these captures: rows %.3f px)", st.flash->row_rms_px);
                if (ImGui::Button("Save")) {
                    if (auto r = ctl.save_result(); !r) error = r.error().message;
                }
                ImGui::SameLine();
                ImGui::BeginDisabled(!ctl.connected() || st.running);
                if (ImGui::Button("Write to scanner...")) {
                    write_plan = ctl.plan_write();
                    write_confirmed = false;
                    write_result.clear();
                    ImGui::OpenPopup("Write calibration to scanner");
                }
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Stores this calibration in the scanner, replacing the current one, as EXStar's calibration does. "
                                      "A backup of the current one is saved first.");
                if (!st.saved_path.empty()) ImGui::TextWrapped("Saved %s", st.saved_path.c_str());
            }
            if (ctl.connected()) {
                if (ImGui::Button("Restore a backup...")) {
                    restore_path = choose_file();
                    if (!restore_path.empty()) {
                        write_result.clear();
                        ImGui::OpenPopup("Restore calibration backup");
                    }
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Puts a flash-backup-*.bin saved before a write back into the scanner.");
            }
            // ---- write confirmation ----
            ImGui::SetNextWindowSize(ImVec2(640, 0), ImGuiCond_Appearing);
            if (ImGui::BeginPopupModal("Write calibration to scanner", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                if (!write_plan.error.empty()) {
                    ImGui::TextColored(ImVec4(1, 0.45f, 0.35f, 1), "%s", write_plan.error.c_str());
                } else {
                    ImGui::TextWrapped("Replaces the calibration stored in the scanner (currently %s) with this one, dated %s. Only the "
                                       "quick-calibration section changes, as with EXStar's calibration; the factory, colour and white-balance "
                                       "data stay as they are.",
                                       ctl.flash_time().c_str(), write_plan.update ? write_plan.update->calibration_time.c_str() : "-");
                    ImGui::Separator();
                    for (const auto& g : write_plan.gates)
                        ImGui::TextColored(g.ok ? ImVec4(0.4f, 0.9f, 0.5f, 1) : ImVec4(1, 0.45f, 0.35f, 1), "%s  %s", g.ok ? "ok  " : "FAIL", g.what.c_str());
                    ImGui::Separator();
                    if (write_plan.update) {
                        std::string pages;
                        for (const int pg : write_plan.update->pages) pages += std::format("{}{}", pages.empty() ? "" : ", ", pg);
                        ImGui::Text("Flash page(s) written: %s (4 KB each, erased and rewritten, then read back)", pages.empty() ? "none" : pages.c_str());
                    }
                    ImGui::Text("Extrinsics refer to the board in %s", write_plan.reference_view.c_str());
                    ImGui::TextWrapped("Backup of the current calibration: %s", write_plan.backup_path.c_str());
                    ImGui::TextDisabled("Keep the scanner connected until this finishes (a second or two).");
                    if (ctl.emulated()) ImGui::TextColored(ImVec4(0.6f, 0.8f, 1, 1), "Emulator: writes the emulated flash only.");
                    ImGui::BeginDisabled(!write_plan.ready());
                    ImGui::Checkbox("Replace the scanner's calibration", &write_confirmed);
                    ImGui::EndDisabled();
                }
                if (!write_result.empty()) ImGui::TextWrapped("%s", write_result.c_str());
                ImGui::BeginDisabled(!write_plan.ready() || !write_confirmed || !write_result.empty());
                if (ImGui::Button("Write", ImVec2(120, 0))) {
                    auto r = ctl.write_to_scanner(write_plan);
                    write_result = r ? *r : "FAILED: " + r.error().message;
                }
                ImGui::EndDisabled();
                ImGui::SameLine();
                if (ImGui::Button(write_result.empty() ? "Cancel" : "Close", ImVec2(120, 0))) ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
            }
            // ---- restore confirmation ----
            if (ImGui::BeginPopupModal("Restore calibration backup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::TextWrapped("Put %s back into the scanner (currently %s)?", restore_path.c_str(), ctl.flash_time().c_str());
                if (!write_result.empty()) ImGui::TextWrapped("%s", write_result.c_str());
                ImGui::BeginDisabled(!write_result.empty());
                if (ImGui::Button("Restore", ImVec2(120, 0))) {
                    auto r = ctl.restore_backup(restore_path);
                    write_result = r ? *r : "FAILED: " + r.error().message;
                }
                ImGui::EndDisabled();
                ImGui::SameLine();
                if (ImGui::Button(write_result.empty() ? "Cancel" : "Close", ImVec2(120, 0))) ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
            }
            ImGui::Separator();
            if (ImGui::Button("Load captures folder...")) {
                if (const auto dir = choose_directory(); !dir.empty()) {
                    if (auto r = ctl.load_folder(dir); !r) error = r.error().message;
                    else error.clear(), tab_request = 1;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("New session")) ctl.clear_all();
            if (!ctl.connected()) {
                if (ImGui::Button("Reference calibration...")) {
                    if (const auto p = choose_path(); !p.empty()) {
                        if (auto r = ctl.load_reference(p); !r) error = r.error().message;
                        else error.clear();
                    }
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Without a scanner: the calibration to compare with - EXStar's CCF folder, a flash dump (.bin) or a calibration file");
                if (ctl.flash_rig()) {
                    ImGui::SameLine();
                    ImGui::TextDisabled("(%s)", ctl.flash_time().c_str());
                }
            }
            if (!ctl.session_dir().empty()) ImGui::TextDisabled("%s", ctl.session_dir().c_str());
            if (ctl.connected() && ImGui::CollapsingHeader("Lighting")) {
                bool changed = false;
                changed |= ImGui::SliderInt("Exposure", &ctl.lighting.exposure, 300, 8000);
                changed |= ImGui::SliderInt("Gain %", &ctl.lighting.gain, 16, 800);
                changed |= ImGui::SliderInt("Ring light", &ctl.lighting.ring_light, 0, 9000);
                changed |= ImGui::SliderInt("White LEDs", &ctl.lighting.white_leds, 0, 9000);
                if (changed) (void)ctl.apply_lighting();
                ImGui::TextDisabled("EXStar: exposure 1500, gain 400, ring 1000, white 300");
            }
            ImGui::End();

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
            if (offscreen_mode && !snapshot_path) std::this_thread::sleep_for(std::chrono::milliseconds(16));  // headless agent: ~60 Hz

            if (snapshot_path) {
                std::this_thread::sleep_for(std::chrono::milliseconds(16));
                bool ready = false;
                if (snapshot_complete) {
                    if ((captured == static_cast<int>(ctl.plan().size()) || (!load_dir.empty() && captured >= 4)) && !solve_requested) {
                        ctl.solve();
                        solve_requested = true;
                        if (snapshot_tab.empty()) tab_request = 2;
                    }
                    ready = solve_requested && !st.running && (st.ours || !st.error.empty());
                    if (clock.elapsed_ms() > 240000) ready = true;
                } else {
                    ready = clock.elapsed_ms() > snapshot_seconds * 1000.0;
                }
                if (ready && snapshot_frames < 0 && snapshot_write && st.ours) {
                    snapshot_write = false;
                    const auto plan = ctl.plan_write();
                    for (const auto& g : plan.gates) std::println("gate {}: {}", g.ok ? "ok  " : "FAIL", g.what);
                    if (!plan.error.empty()) std::println("plan: {}", plan.error);
                    const auto r = ctl.write_to_scanner(plan);
                    std::println("write: {}", r ? *r : "FAILED: " + r.error().message);
                    if (ctl.flash_rig()) {
                        const auto d = calibrate::compare_calibrations(st.ours->rig, *ctl.flash_rig());
                        std::println("scanner now holds {}: vs the solve, rig {:.2e} deg, ray mapping {:.2e} / {:.2e} px", ctl.flash_time(),
                                     d.rotation_deg.norm(), d.left.mapping_px, d.right.mapping_px);
                    }
                }
                if (ready && snapshot_frames < 0) {
                    snapshot_frames = 3;  // a few frames for the layout to settle
                    if (snapshot_tab == "coverage") tab_request = 1;
                    if (snapshot_tab == "live") tab_request = 0;
                }
                if (snapshot_frames > 0 && --snapshot_frames == 0) {
                    [cmd waitUntilCompleted];
                    if (st.ours)
                        std::println("solved: rms {:.3f} px, rows {:.3f} px{}", st.ours->rms_px, st.ours->row_rms_px,
                                     st.flash ? std::format("; flash on these captures: rows {:.3f} px", st.flash->row_rms_px) : "");
                    if (st.vs_flash)
                        std::println("ours - flash: rig rotation {:.3f} deg, right cy {:+.2f} fx {:+.2f}", st.vs_flash->rotation_deg.norm(), st.vs_flash->right.dcy,
                                     st.vs_flash->right.dfx);
                    std::println("captured {} in {:.1f} s; snapshot: {}", captured, clock.elapsed_ms() / 1000.0, write_png(queue, offscreen, snapshot_path) ? snapshot_path : "FAILED");
                    break;
                }
            }
        }
    }
    ctl.disconnect();
    ImGui_ImplMetal_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
