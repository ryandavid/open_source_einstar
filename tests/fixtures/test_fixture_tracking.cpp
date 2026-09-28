#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <print>

#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/track/icp.hpp"
#include "einstar/track/tsdf.hpp"

using namespace einstar;

TEST_CASE("fixture frames: model raycast and ICP between consecutive frames") {
    const auto dir = std::filesystem::path(std::getenv("HOME")) / "Documents/EXStar/mustang_differential";
    if (!std::filesystem::exists(dir / "Project1.data_base")) SKIP("mustang fixture not available");
    auto proj = fixtures::ExstarProject::open(dir / "Project1.ir_E10_prj");
    REQUIRE(proj.has_value());
    auto f0 = (*proj)->read_frame(0);
    auto f1 = (*proj)->read_frame(1);
    REQUIRE((f0 && f1));
    const track::Intrinsics k{640, 512, f0->intrinsics.fx, f0->intrinsics.fy, f0->intrinsics.cx, f0->intrinsics.cy};
    const auto d0 = track::make_depth_frame(f0->depth, k);
    const auto d1 = track::make_depth_frame(f1->depth, k);
    int valid_depth = 0;
    float zmin = 1e9f, zmax = 0;
    for (const auto& p : d0.points.pixels())
        if (p.z() > 0) {
            ++valid_depth;
            zmin = std::min(zmin, p.z());
            zmax = std::max(zmax, p.z());
        }
    std::println("frame0: {} valid depth px, z {:.1f}..{:.1f}", valid_depth, zmin, zmax);

    track::TsdfVolume vol;
    vol.integrate(d0, f0->T_world_camera);
    const auto mk = k.scaled(0.5);
    const auto ray = vol.raycast(f0->T_world_camera, mk);
    int valid = 0;
    for (auto v : ray.valid.pixels()) valid += v;
    std::println("bricks {}, raycast valid {} of {}x{}", vol.brick_count(), valid, mk.width, mk.height);
    REQUIRE(valid > 1000);

    const auto r = track::icp_point_to_plane(d1, ray, f0->T_world_camera, f0->T_world_camera, {});
    const SE3 err = f1->T_world_camera.inverse() * r.T_world_camera;
    std::println("icp converged {} n {} rms {:.3f} inl {:.2f}; vs EXStar {:.3f} mm {:.3f} deg", r.converged,
                 r.correspondences, r.rms_mm, r.inlier_ratio, translation_norm(err), rotation_angle(err) * 180 / M_PI);
    REQUIRE(r.converged);
}
