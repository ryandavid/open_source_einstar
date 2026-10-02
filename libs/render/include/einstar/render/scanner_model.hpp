#pragma once

// A low-poly Einstar, in the scanner frame (x towards the right camera, y down the image, z forward; mm;
// origin midway between the IR cameras on the front face). Held upright, x is up: a 220 x 46 x 55 mm bar,
// black glass in front with the right IR camera at the top (x +80), the texture camera below it (+43), the
// projector near the middle and the left IR camera at the bottom (-80); a blue-grey shell whose back
// narrows towards the middle, where it is held. No Metal or ImGui: the scanning app's renderer and the
// calibration app's board view both draw it.

#include <array>
#include <cmath>
#include <numbers>
#include <vector>

#include <Eigen/Core>

#include "einstar/render/types.hpp"

namespace einstar::render {

struct ModelFace {
    std::vector<Eigen::Vector3f> v;  // convex and planar, counter-clockwise seen from outside
    Rgba8 color;
};

struct ScannerModel {
    std::vector<ModelFace> body;   // the closed shell
    std::vector<ModelFace> front;  // on the front glass (lenses, projector, LEDs), each layer a little further
                                   // out: drawn in order after the body, they stack correctly
};

[[nodiscard]] inline const ScannerModel& scanner_model() {
    static const ScannerModel model = [] {
        using V = Eigen::Vector3f;
        constexpr Rgba8 kGlass{18, 20, 24, 255}, kBezel{40, 44, 50, 255};
        constexpr Rgba8 kShell{140, 168, 186, 255}, kShellBack{96, 116, 130, 255};
        ScannerModel m;
        // Cross-section (y, z) at stations along x; the back (z) comes forward at the grip and the ends round
        // off. Eight edges: front glass, front bevels, sides, back bevels, back. Clockwise seen from +x.
        struct Station {
            float x, scale, back;
        };
        const std::array<Station, 8> st = {{{-110, 0.8f, -46}, {-104, 1, -52}, {-80, 1, -52}, {-40, 1, -40},
                                            {40, 1, -40}, {80, 1, -52}, {104, 1, -52}, {110, 0.8f, -46}}};
        auto section = [](const Station& s) {
            const float hw = 23 * s.scale, b = 6 * s.scale, z0 = s.scale < 1 ? -2.0f : 0.0f;
            return std::array<V, 8>{V(s.x, -hw + b, z0),     V(s.x, hw - b, z0),       V(s.x, hw, z0 - b),       V(s.x, hw, s.back + b),
                                    V(s.x, hw - b, s.back), V(s.x, -hw + b, s.back), V(s.x, -hw, s.back + b), V(s.x, -hw, z0 - b)};
        };
        const std::array<Rgba8, 8> edge_color = {kGlass, kBezel, kShell, kShellBack, kShellBack, kShellBack, kShell, kBezel};
        for (std::size_t i = 0; i + 1 < st.size(); ++i) {
            const auto a = section(st[i]), b = section(st[i + 1]);
            for (std::size_t e = 0; e < 8; ++e) {
                const std::size_t n = (e + 1) % 8;
                m.body.push_back({{a[e], b[e], b[n], a[n]}, edge_color[e]});
            }
        }
        // Caps: the section as is faces -x (the bottom), reversed it faces +x (the top).
        const auto lo = section(st.front()), hi = section(st.back());
        m.body.push_back({{lo.begin(), lo.end()}, kShell});
        m.body.push_back({{hi.rbegin(), hi.rend()}, kShell});

        // Front details, facing +z.
        auto disc = [&](float x, float y, float z, float r, int n, Rgba8 c) {
            ModelFace f{{}, c};
            for (int i = 0; i < n; ++i) {
                const float t = 2 * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(n);
                f.v.emplace_back(x + r * std::cos(t), y + r * std::sin(t), z);
            }
            m.front.push_back(std::move(f));
        };
        for (const auto& [x, r] : {std::pair{80.0f, 8.0f}, std::pair{43.0f, 6.0f}, std::pair{-80.0f, 8.0f}}) {
            disc(x, 0, 0.3f, r + 2.5f, 24, {32, 36, 42, 255});  // bezel
            disc(x, 0, 0.4f, r + 0.8f, 24, {110, 120, 135, 255});  // rim
            disc(x, 0, 0.5f, r, 24, {8, 10, 14, 255});             // lens
        }
        for (const auto& [x, y] : {std::pair{3.0f, -6.0f}, std::pair{3.0f, 6.0f}, std::pair{-9.0f, 0.0f}})  // projector
            m.front.push_back({{V(x - 2.5f, y - 2.5f, 0.3f), V(x + 2.5f, y - 2.5f, 0.3f), V(x + 2.5f, y + 2.5f, 0.3f), V(x - 2.5f, y + 2.5f, 0.3f)},
                               {60, 52, 48, 255}});
        for (const float x : {80.0f, 43.0f, -80.0f})  // LEDs around each camera
            for (const float dx : {-11.0f, 11.0f})
                for (const float y : {-12.0f, 12.0f}) disc(x + dx, y, 0.3f, 1.5f, 8, {150, 155, 165, 255});
        return m;
    }();
    return model;
}

}  // namespace einstar::render
