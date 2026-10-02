#include "board_view.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>

namespace einstar::app {
namespace {

constexpr ImU32 kGreen = IM_COL32(60, 220, 100, 255);
constexpr ImU32 kAmber = IM_COL32(250, 180, 40, 255);
constexpr ImU32 kCyan = IM_COL32(70, 200, 255, 255);
constexpr ImU32 kGreyScanner = IM_COL32(120, 120, 128, 200);
constexpr std::array<ImU32, 5> kGroup = {IM_COL32(90, 200, 255, 255), IM_COL32(255, 140, 90, 255), IM_COL32(200, 120, 255, 255),
                                         IM_COL32(120, 230, 120, 255), IM_COL32(255, 220, 80, 255)};
constexpr std::array<const char*, 5> kGroupName = {"face-on", "right edge near", "left edge near", "bottom edge near", "top edge near"};
constexpr double kDeg = M_PI / 180.0;

ImU32 with_alpha(ImU32 c, int a) { return (c & 0x00FFFFFFu) | (static_cast<ImU32>(std::clamp(a, 0, 255)) << IM_COL32_A_SHIFT); }

// Perspective camera looking at the board.
struct ViewCam {
    Vec3 eye, right, up, fwd;
    double focal = 1;
    ImVec2 centre;

    struct P {
        ImVec2 xy;
        double depth = 0;
        bool ok = false;
    };
    [[nodiscard]] P project(const Vec3& p) const {
        const Vec3 d = p - eye;
        const double z = d.dot(fwd);
        if (z < 20) return {};
        return {ImVec2(centre.x + static_cast<float>(focal * d.dot(right) / z), centre.y - static_cast<float>(focal * d.dot(up) / z)), z, true};
    }
    [[nodiscard]] float scale_at(double depth) const { return static_cast<float>(focal / std::max(depth, 1.0)); }  // px per mm
};

void line3(ImDrawList* dl, const ViewCam& c, const Vec3& a, const Vec3& b, ImU32 col, float w) {
    const auto pa = c.project(a), pb = c.project(b);
    if (pa.ok && pb.ok) dl->AddLine(pa.xy, pb.xy, col, w);
}

void dashed3(ImDrawList* dl, const ViewCam& c, const Vec3& a, const Vec3& b, ImU32 col, float w, int n = 16) {
    for (int i = 0; i < n; i += 2) line3(dl, c, a + (b - a) * (i / double(n)), a + (b - a) * ((i + 1) / double(n)), col, w);
}

// A circle of radius r around `centre`, in the plane with normal `n`.
void circle3(ImDrawList* dl, const ViewCam& c, const Vec3& centre, const Vec3& n, double r, ImU32 col, float w, ImU32 fill = 0) {
    const Vec3 a = n.unitOrthogonal(), b = n.cross(a).normalized();
    std::array<ImVec2, 32> pts;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const double t = 2 * M_PI * static_cast<double>(i) / pts.size();
        const auto p = c.project(centre + r * (std::cos(t) * a + std::sin(t) * b));
        if (!p.ok) return;
        pts[i] = p.xy;
    }
    if (fill) dl->AddConvexPolyFilled(pts.data(), static_cast<int>(pts.size()), fill);
    dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), col, ImDrawFlags_Closed, w);
}

// Where a ray from `o` along `d` meets the board plane (z = 0), if ahead.
std::optional<Vec3> on_board_plane(const Vec3& o, const Vec3& d) {
    if (std::abs(d.z()) < 1e-9) return std::nullopt;
    const double t = -o.z() / d.z();
    if (t <= 0) return std::nullopt;
    return o + t * d;
}

// The scanner: a body across the camera baseline, a handle, the two cameras, and (live) its field of view
// on the board and its aim line. Scanner frame: x to the right camera, y down the image, z forward (mm).
void draw_scanner(ImDrawList* dl, const ViewCam& c, const SE3& B_S, ImU32 col, float w, bool live, bool aim_ok) {
    auto at = [&](double x, double y, double z) { return B_S * Vec3(x, y, z); };
    auto box = [&](double x0, double x1, double y0, double y1, double z0, double z1) {
        const std::array<Vec3, 8> v = {at(x0, y0, z0), at(x1, y0, z0), at(x1, y1, z0), at(x0, y1, z0),
                                       at(x0, y0, z1), at(x1, y0, z1), at(x1, y1, z1), at(x0, y1, z1)};
        for (std::size_t i = 0; i < 4; ++i) {
            line3(dl, c, v[i], v[(i + 1) % 4], col, w);
            line3(dl, c, v[i + 4], v[(i + 1) % 4 + 4], col, w);
            line3(dl, c, v[i], v[i + 4], col, w);
        }
    };
    box(-100, 100, -32, 32, -55, 0);   // the head, cameras on its front face (z = 0)
    box(-22, 22, 32, 170, -48, -10);   // the handle, towards the image's bottom
    for (const double x : {-80.0, 80.0}) {
        const auto p = c.project(at(x, 0, 0));
        if (p.ok) dl->AddCircleFilled(p.xy, std::max(2.5f, 9 * c.scale_at(p.depth)), col, 12);
    }
    if (!live) return;
    // Field of view of the left camera (about 58 x 48 degrees) where it meets the board plane.
    const Vec3 o = at(0, 0, 0);
    std::array<ImVec2, 4> fp{};
    bool all = true;
    const double tx = std::tan(29 * kDeg), ty = std::tan(24 * kDeg);
    const std::array<Vec2, 4> corners = {Vec2(-tx, -ty), Vec2(tx, -ty), Vec2(tx, ty), Vec2(-tx, ty)};
    for (std::size_t i = 0; i < 4; ++i) {
        const auto hit = on_board_plane(o, B_S.linear() * Vec3(corners[i].x(), corners[i].y(), 1));
        const auto p = hit ? c.project(*hit) : ViewCam::P{};
        if (!p.ok) all = false;
        else fp[i] = p.xy;
        if (hit) line3(dl, c, o, *hit, with_alpha(col, 30), 1.0f);
    }
    if (all) dl->AddPolyline(fp.data(), 4, with_alpha(col, 90), ImDrawFlags_Closed, 1.0f);
    // Aim line to the board, and where it lands (green on the centre).
    if (const auto hit = on_board_plane(o, B_S.linear().col(2))) {
        line3(dl, c, o, *hit, col, 2.0f);
        const auto p = c.project(*hit);
        if (p.ok) dl->AddCircle(p.xy, 7, aim_ok ? kGreen : kAmber, 16, 2.5f);
    }
}

}  // namespace

void BoardView::draw(const BoardViewInput& in, ImVec2 size) {
    const calibrate::BoardSpec board;
    const Vec3 centre = board.centre();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);

    // Interaction: drag orbits, wheel zooms, double-click resets.
    ImGui::InvisibleButton("##board_view", size);
    const bool hovered = ImGui::IsItemHovered();
    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        const ImVec2 d = ImGui::GetIO().MouseDelta;
        yaw_ -= d.x * 0.006;
        pitch_ = std::clamp(pitch_ + d.y * 0.006, -1.2, 1.2);
    }
    if (hovered && ImGui::GetIO().MouseWheel != 0) zoom_ = std::clamp(zoom_ * std::pow(0.9, ImGui::GetIO().MouseWheel), 0.4, 3.0);
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) reset_view();

    dl->PushClipRect(p0, p1, true);
    dl->AddRectFilled(p0, p1, IM_COL32(18, 20, 24, 255));

    // Operator view: from where you stand -- towards the image's bottom side of a face-on scanner (assumed to
    // be the operator's side), a little to the right and well above the board -- with the board's normal
    // (away from the table) as up, so moving the scanner right moves it right on screen.
    calibrate::PoseTarget home;
    const SE3 B_home = calibrate::board_from_scanner(home, board);
    const Vec3 eye0 = centre + B_home.linear() * (Vec3(0.2, 0.75, -0.62).normalized() * 1500.0);
    const Vec3 up0 = -Vec3::UnitZ();
    const Vec3 right0 = (centre - eye0).cross(up0).normalized();
    const Mat3 orbit = (Eigen::AngleAxisd(yaw_, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch_, right0)).toRotationMatrix();
    ViewCam cam;
    cam.eye = centre + zoom_ * (orbit * (eye0 - centre));
    cam.fwd = (centre - cam.eye).normalized();
    cam.right = cam.fwd.cross(orbit * up0).normalized();
    cam.up = cam.right.cross(cam.fwd);
    cam.focal = 0.5 * size.y / std::tan(0.5 * 48 * kDeg);
    cam.centre = ImVec2(p0.x + size.x * 0.42f, p0.y + size.y * 0.55f);  // (the camera inset is at the bottom right)
    const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(ImGui::GetTime()) * 6.0f);

    // ---- the board: its outline, the grid's dots (large ones larger) ----
    {
        const auto o = board.outline();
        std::array<ImVec2, 4> q{};
        bool ok = true;
        for (std::size_t i = 0; i < 4; ++i) {
            const auto p = cam.project(o[i]);
            ok = ok && p.ok;
            q[i] = p.xy;
        }
        if (ok) {
            dl->AddQuadFilled(q[0], q[1], q[2], q[3], IM_COL32(58, 60, 66, 255));
            dl->AddQuad(q[0], q[1], q[2], q[3], IM_COL32(150, 150, 158, 255), 1.5f);
        }
        for (int r = 0; r < board.rows; ++r)
            for (int col = 0; col < board.cols; ++col) {
                const Vec2 g(col, r);
                const auto p = cam.project(board.point(g));
                if (p.ok) dl->AddCircleFilled(p.xy, std::max(1.5f, 2.5f * cam.scale_at(p.depth)), IM_COL32(230, 230, 230, 255), 10);
            }
        for (const auto& g : board.large) {
            const auto p = cam.project(board.point(g));
            if (p.ok) dl->AddCircleFilled(p.xy, std::max(2.5f, 4.5f * cam.scale_at(p.depth)), IM_COL32(255, 255, 255, 255), 12);
        }
    }

    // ---- the plan: a line per group, a dot per distance ----
    const auto& plan = *in.plan;
    struct Dot {
        int index;
        Vec3 pos, dir;
        double depth;
    };
    std::vector<Dot> dots;
    std::array<double, 5> reach{};
    std::array<Vec3, 5> dirs{};
    for (std::size_t i = 0; i < plan.size(); ++i) {
        const auto& t = plan[i];
        const Vec3 pos = calibrate::board_from_scanner(t, board).translation();
        const Vec3 dir = (pos - centre).normalized();
        const auto g = static_cast<std::size_t>(t.group);
        dirs[g] = dir;
        reach[g] = std::max(reach[g], t.distance_mm);
        const auto p = cam.project(pos);
        dots.push_back({static_cast<int>(i), pos, dir, p.ok ? p.depth : 1e9});
    }
    for (std::size_t g = 0; g < 5; ++g) {
        if (reach[g] <= 0) continue;
        line3(dl, cam, centre, centre + dirs[g] * (reach[g] + 70), with_alpha(kGroup[g], 110), 1.5f);
        const auto p = cam.project(centre + dirs[g] * (reach[g] + 90));
        if (p.ok) dl->AddText(ImVec2(p.xy.x - ImGui::CalcTextSize(kGroupName[g]).x * 0.5f, p.xy.y - 8), with_alpha(kGroup[g], 210), kGroupName[g]);
    }
    // Captured views: where the scanner actually was (its camera baseline).
    for (std::size_t i = 0; i < in.ghosts.size(); ++i)
        if (in.ghosts[i]) line3(dl, cam, *in.ghosts[i] * Vec3(-80, 0, 0), *in.ghosts[i] * Vec3(80, 0, 0), IM_COL32(60, 220, 100, 90), 2.0f);
    // Far dots first.
    std::ranges::sort(dots, [](const Dot& a, const Dot& b) { return a.depth > b.depth; });
    for (const auto& d : dots) {
        const auto& t = plan[static_cast<std::size_t>(d.index)];
        const auto g = static_cast<std::size_t>(t.group);
        const bool done = static_cast<std::size_t>(d.index) < in.captured.size() && in.captured[static_cast<std::size_t>(d.index)];
        const bool next = d.index == in.target;
        const auto p = cam.project(d.pos);
        if (!p.ok) continue;
        // Acceptance: a disc across the line (the tilt tolerance at this distance) and the distance band along it.
        const double lateral = t.distance_mm * std::tan(t.tilt_tol_deg * kDeg);
        if (next) {
            const ImU32 col = in.in_position ? kAmber : kCyan;
            circle3(dl, cam, d.pos, d.dir, lateral, with_alpha(col, 230), 2.0f, with_alpha(col, 40 + static_cast<int>(40 * pulse)));
            line3(dl, cam, d.pos - d.dir * t.distance_tol_mm, d.pos + d.dir * t.distance_tol_mm, col, 4.0f);
            const auto label = std::format("{} {:.0f} mm", kGroupName[g], t.distance_mm);
            dl->AddText(ImVec2(p.xy.x + 14, p.xy.y - 20), col, label.c_str());
            if (in.in_position) {
                // Hold-still progress around the dot.
                dl->PathArcTo(p.xy, 16, -1.5708f, -1.5708f + static_cast<float>(std::clamp(in.steady_frac, 0.0, 1.0)) * 6.2832f, 32);
                dl->PathStroke(kGreen, 0, 4);
            }
        }
        const ImU32 fill = done ? kGreen : next ? (in.in_position ? kAmber : kCyan) : with_alpha(kGroup[g], 120);
        const float r = next ? 6.0f + 2.5f * pulse : done ? 6.0f : 4.5f;
        dl->AddCircleFilled(p.xy, r, fill, 16);
        if (done) dl->AddCircle(p.xy, r + 2, IM_COL32(20, 60, 30, 255), 16, 1.0f);
    }

    // ---- the scanner now (or where it was last seen) ----
    if (in.scanner) last_scanner_ = in.scanner;
    if (last_scanner_) {
        const bool live = in.scanner.has_value();
        const ImU32 col = !live ? kGreyScanner : in.in_position ? kGreen : kAmber;
        draw_scanner(dl, cam, *last_scanner_, col, live ? 2.0f : 1.2f, live, in.aim_ok);
        // Towards the target: from the scanner to its dot.
        if (live && in.target >= 0 && !in.in_position) {
            const Vec3 goal = calibrate::board_from_scanner(plan[static_cast<std::size_t>(in.target)], board).translation();
            dashed3(dl, cam, last_scanner_->translation(), goal, with_alpha(kCyan, 200), 1.5f, 20);
        }
        if (!live) {
            const auto p = cam.project(last_scanner_->translation());
            if (p.ok) dl->AddText(ImVec2(p.xy.x + 12, p.xy.y + 8), IM_COL32(170, 170, 170, 255), "board not in view (last seen here)");
        }
    }

    // ---- legend ----
    {
        const ImVec2 l(p0.x + 12, p1.y - 24);
        float x = l.x;
        auto item = [&](ImU32 c, const char* s) {
            dl->AddCircleFilled(ImVec2(x + 6, l.y + 8), 5, c, 12);
            dl->AddText(ImVec2(x + 16, l.y), IM_COL32(170, 170, 170, 255), s);
            x += 26 + ImGui::CalcTextSize(s).x;
        };
        item(with_alpha(kGroup[0], 120), "to do");
        item(kCyan, "next");
        item(kAmber, "in position: hold still");
        item(kGreen, "captured");
        dl->AddText(ImVec2(x + 10, l.y), IM_COL32(120, 120, 120, 255), "drag: orbit   scroll: zoom   double-click: reset view");
    }
    dl->PopClipRect();
}

}  // namespace einstar::app
