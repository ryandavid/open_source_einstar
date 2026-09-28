#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <print>

#include "einstar/core/timing.hpp"
#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/track/global_registration.hpp"
#include "einstar/track/icp.hpp"
#include "einstar/track/tsdf.hpp"

using namespace einstar;

TEST_CASE("global registration finds fixture frames in a model built from other frames") {
    const auto dir = std::filesystem::path(std::getenv("HOME")) / "Documents/EXStar/mustang_differential";
    if (!std::filesystem::exists(dir / "Project1.data_base")) SKIP("mustang fixture not available");
    auto proj = fixtures::ExstarProject::open(dir / "Project1.ir_E10_prj");
    REQUIRE(proj.has_value());

    track::TsdfVolume vol;
    track::Intrinsics k;
    for (std::size_t i = 0; i < 480; i += 4) {
        auto f = (*proj)->read_frame(i);
        REQUIRE(f.has_value());
        k = {640, 512, f->intrinsics.fx, f->intrinsics.fy, f->intrinsics.cx, f->intrinsics.cy};
        vol.integrate(track::make_depth_frame(f->depth, k), f->T_world_camera);
    }
    track::OrientedCloud model;
    for (const auto& sp : vol.extract_points(0, 1.0f)) {
        model.points.push_back(sp.position);
        model.normals.push_back(sp.normal);
    }
    track::GlobalRegistrationParams gp;
    Stopwatch sw;
    track::FeatureModel fm(track::voxel_downsample(model, gp.voxel_mm), gp.feature_radius_mm);
    std::println("model: {} points, {} downsampled, features built in {:.0f} ms", model.points.size(), fm.cloud().points.size(),
                 sw.elapsed_ms());

    int found = 0, correct = 0, tried = 0;
    for (std::size_t i = 2; i < 480; i += 40) {  // frames not used for the model (offset by 2)
        auto f = (*proj)->read_frame(i);
        REQUIRE(f.has_value());
        const auto df = track::make_depth_frame(f->depth, k);
        track::OrientedCloud cloud;
        for (int v = 0; v < df.points.height(); v += 2)
            for (int u = 0; u < df.points.width(); u += 2)
                if (df.normals(u, v).squaredNorm() > 0) {
                    cloud.points.push_back(df.points(u, v));
                    cloud.normals.push_back(df.normals(u, v));
                }
        ++tried;
        sw.reset();
        const auto r = track::register_global(cloud, fm, gp, static_cast<std::uint32_t>(i));
        const double ms = sw.elapsed_ms();
        if (!r) {
            std::println("frame {:4}: no match ({:.0f} ms)", i, ms);
            continue;
        }
        ++found;
        const SE3 coarse = f->T_world_camera.inverse() * r->T_model_frame;
        // Refine exactly as the tracker does after a global candidate.
        const auto mk = k.scaled(0.5);
        const auto model_view = vol.raycast(r->T_model_frame, mk);
        const auto icp = track::icp_point_to_plane(df, model_view, r->T_model_frame, r->T_model_frame, {});
        const SE3 err = f->T_world_camera.inverse() * icp.T_world_camera;
        const bool ok = icp.converged && translation_norm(err) < 1.0 && rotation_angle(err) * 180 / M_PI < 0.3;
        correct += ok;
        std::println("frame {:4}: coarse {:5.1f} mm {:4.1f} deg -> icp {:5.2f} mm {:5.2f} deg (inl {:.2f} eig {:.1e}) {} ({:.0f} ms)",
                     i, translation_norm(coarse), rotation_angle(coarse) * 180 / M_PI, translation_norm(err),
                     rotation_angle(err) * 180 / M_PI, icp.inlier_ratio, icp.min_eigenvalue_ratio, ok ? "" : "WRONG", ms);
    }
    std::println("global registration: {}/{} found, {} correct", found, tried, correct);
    CHECK(correct >= tried / 2);
}
