// Marker stickers leave holes in the speckle depth; the frontend fills them from the surrounding
// surface so the model has no dimples.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <print>
#include <random>

#include "einstar/pipeline/stereo_frontend.hpp"
#include "synthetic_setup.hpp"

using namespace einstar;

TEST_CASE("marker depth holes are filled on the surface") {
    const auto rig = e2e::einstar_like_rig();
    auto setup = e2e::make_scene();
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> ux(-260, 120), uz(-120, 170);
    for (int tries = 0; tries < 20000 && setup.scene.markers.size() < 45; ++tries) {
        const Vec3 c(ux(rng), 70.0, uz(rng));
        bool ok = true;
        for (const auto& m : setup.scene.markers) ok = ok && (m.center - c).norm() > 24.0;
        if (ok) setup.scene.markers.push_back({c, Vec3(0, -1, 0), 6.0, 10.0});
    }
    const std::uint32_t id = 30;
    ImageU8 l, r;
    e2e::render_sensor(setup, rig, e2e::truth_pose(id), 0, 1, l);
    e2e::render_sensor(setup, rig, e2e::truth_pose(id), 1, 2, r);

    auto run = [&](bool fill) {
        pipeline::StereoFrontendParams p;
        p.fill_marker_holes = fill;
        p.backend = pipeline::StereoBackend::cpu;
        pipeline::StereoFrontend fe(rig, p);
        auto out = fe.process(l, r);
        out.frame.ensure_cpu();
        const SE3 T = e2e::truth_pose(id) * fe.rectification().T_left_rectified();
        int holes = 0, on_plane = 0, total = 0;
        double max_err = 0;
        const int W = out.frame.width(), H = out.frame.height();
        const double full_w = fe.rectification().rectified.width;
        int used = 0;
        for (const auto& m : out.markers) {
            // Pixels inside the sticker disc (half-resolution depth grid); only stickers inside both
            // rectified views have surroundings with depth.
            const double cx = m.left_rect.x() * 0.5, cy = m.left_rect.y() * 0.5;
            const double rad = 0.5 * 3.0 * fe.rectification().geometry.f / m.position.z() * 0.5;  // 3 mm radius, half res
            const double margin = 4 * rad;
            if (cx - margin < 0 || cy - margin < 0 || cx + margin >= W || cy + margin >= H) continue;
            if (m.right_rect.x() - 2 * margin < 0 || m.right_rect.x() + 2 * margin >= full_w) continue;
            ++used;
            for (int y = static_cast<int>(cy - rad); y <= static_cast<int>(cy + rad); ++y)
                for (int x = static_cast<int>(cx - rad); x <= static_cast<int>(cx + rad); ++x) {
                    if ((x - cx) * (x - cx) + (y - cy) * (y - cy) > rad * rad) continue;
                    ++total;
                    const Vec3f& p3 = out.frame.points(x, y);
                    if (p3.z() <= 0) {
                        ++holes;
                        continue;
                    }
                    const double err = std::abs((T * p3.cast<double>()).y() - 70.0);
                    max_err = std::max(max_err, err);
                    on_plane += err < 0.2;
                }
        }
        return std::tuple{used, total, holes, on_plane, max_err};
    };
    const auto [n0, t0, h0, p0, e0] = run(false);
    const auto [n1, t1, h1, p1, e1] = run(true);
    // Without filling the sticker's depth is missing or guessed by stereo (off the surface); with it,
    // the disc lies on the surface.
    std::println("marker discs: {} markers, {} pixels; without filling {} empty + {} off the surface, with filling {} empty + {} off "
                 "(max error {:.3f} mm)",
                 n1, t1, h0, t0 - h0 - p0, h1, t1 - h1 - p1, e1);
    REQUIRE(n1 >= 4);
    CHECK(t0 - p0 > t0 / 5);   // the defect is real
    CHECK(h1 == 0);
    CHECK(p1 >= t1 * 97 / 100);
    (void)e0;
}
