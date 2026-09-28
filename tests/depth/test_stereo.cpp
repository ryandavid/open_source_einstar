#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <print>
#include <vector>

#include "einstar/core/timing.hpp"
#include "einstar/depth/point_image.hpp"
#include "einstar/depth/stereo.hpp"
#include "einstar/synth/speckle_scene.hpp"

using namespace einstar;

namespace {

struct Rig {
    CameraModel cam;
    SE3 T_world_left = SE3::Identity();
    SE3 T_world_right = SE3::Identity();
    synth::Projector projector;
    depth::RectifiedGeometry geom;
};

// Rectified half-resolution rig roughly matching the Einstar's geometry.
Rig make_rig(double baseline) {
    Rig rig;
    rig.cam.width = 640;
    rig.cam.height = 512;
    rig.cam.fx = rig.cam.fy = 582.0;
    rig.cam.cx = 320.0;
    rig.cam.cy = 256.0;
    rig.T_world_right.translation() = Vec3(baseline, 0, 0);
    rig.projector.model = rig.cam;
    rig.projector.model.width = 1280;
    rig.projector.model.height = 800;
    rig.projector.model.fx = rig.projector.model.fy = 900.0;
    rig.projector.model.cx = 640.0;
    rig.projector.model.cy = 400.0;
    rig.projector.T_world_projector.translation() = Vec3(baseline / 2, 15.0, 0);
    rig.projector.pattern = synth::DotPattern::random(1280, 800, 60000, 1.1, 42);
    rig.geom = {rig.cam.fx, rig.cam.cx, rig.cam.cy, baseline};
    return rig;
}

struct Metrics {
    double valid_fraction = 0;
    double median_abs_mm = 0;
    double rms_mm = 0;
    double outlier_fraction = 0;  // |dz| > 1 mm among valid
};

// Only pixels whose true correspondence lies inside the right image are scored.
Metrics evaluate(const depth::PointImage& pts, const ImageF32& truth, const depth::RectifiedGeometry& g, int margin) {
    std::vector<double> errs;
    int gt = 0;
    for (int y = margin; y < truth.height() - margin; ++y)
        for (int x = margin; x < truth.width() - margin; ++x) {
            if (truth(x, y) <= 0) continue;
            if (x - g.disparity_from_depth(truth(x, y)) < margin) continue;
            ++gt;
            const float z = pts.points(x, y).z();
            if (z > 0) errs.push_back(std::abs(z - truth(x, y)));
        }
    Metrics m;
    m.valid_fraction = gt ? static_cast<double>(errs.size()) / gt : 0;
    if (errs.empty()) return m;
    double ss = 0;
    int outliers = 0;
    for (double e : errs) {
        ss += e * e;
        if (e > 1.0) ++outliers;
    }
    m.outlier_fraction = static_cast<double>(outliers) / static_cast<double>(errs.size());
    m.rms_mm = std::sqrt(ss / static_cast<double>(errs.size()));
    std::nth_element(errs.begin(), errs.begin() + errs.size() / 2, errs.end());
    m.median_abs_mm = errs[errs.size() / 2];
    return m;
}

depth::StereoParams params_for(const Rig& rig, double zmin, double zmax, int levels) {
    depth::StereoParams p;
    p.pyramid_levels = levels;
    const double scale = 1.0 / (1 << levels);
    const double dmax = rig.geom.disparity_from_depth(zmin) * scale;
    const double dmin = rig.geom.disparity_from_depth(zmax) * scale;
    p.sgm.min_disparity = std::max(0, static_cast<int>(dmin) - 2);
    p.sgm.num_disparities = static_cast<int>(dmax - dmin) + 6;
    p.speckle.max_region_size = 50;
    return p;
}

}  // namespace

TEST_CASE("stereo recovers a tilted plane and a sphere from synthetic speckle") {
    const Rig rig = make_rig(100.0);
    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 0, 380), Vec3(0.25, -0.15, -1).normalized()});
    scene.primitives.push_back(synth::Sphere{Vec3(20, 10, 300), 45.0});

    synth::RenderParams rp;
    const auto left = synth::render_view(scene, rig.projector, rig.cam, rig.T_world_left, rp);
    rp.seed = 2;
    const auto right = synth::render_view(scene, rig.projector, rig.cam, rig.T_world_right, rp);

    const auto params = params_for(rig, 200.0, 500.0, 2);
    Stopwatch sw;
    const auto result = depth::compute_disparity(left.image.view(), right.image.view(), params);
    const double ms = sw.elapsed_ms();
    const auto pts = depth::disparity_to_points(result.disparity, result.confidence, rig.geom);
    const Metrics m = evaluate(pts, left.depth, rig.geom, 8);
    std::println("stereo 640x512: {:.1f} ms, valid {:.1f}%, median |dz| {:.3f} mm, rms {:.3f} mm, outliers {:.2f}%", ms,
                 100 * m.valid_fraction, m.median_abs_mm, m.rms_mm, 100 * m.outlier_fraction);

    // Occluded / out-of-view regions near the left border can never be matched.
    REQUIRE(m.valid_fraction > 0.85);
    REQUIRE(m.median_abs_mm < 0.15);
    REQUIRE(m.outlier_fraction < 0.01);
}

TEST_CASE("sgm alone on a fronto-parallel plane gives the expected integer disparity") {
    const Rig rig = make_rig(100.0);
    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 0, 400), Vec3(0, 0, -1)});
    const auto left = synth::render_view(scene, rig.projector, rig.cam, rig.T_world_left, {});
    const auto right = synth::render_view(scene, rig.projector, rig.cam, rig.T_world_right, {});
    depth::SgmParams sp;
    sp.min_disparity = 120;
    sp.num_disparities = 64;
    const auto disp = depth::sgm_disparity(left.image.view(), right.image.view(), sp, true);
    const double expected = rig.geom.disparity_from_depth(400.0);  // 145.5
    int good = 0, valid = 0;
    for (int y = 20; y < disp.height() - 20; ++y)
        for (int x = 200; x < disp.width() - 20; ++x) {
            if (disp(x, y) < 0) continue;
            ++valid;
            if (std::abs(disp(x, y) - expected) < 0.5) ++good;
        }
    REQUIRE(valid > 0);
    REQUIRE(static_cast<double>(good) / valid > 0.97);
}
