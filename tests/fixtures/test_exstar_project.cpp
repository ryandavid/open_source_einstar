#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <print>

#include "einstar/fixtures/exstar_project.hpp"
#include "real_data.hpp"

using namespace einstar;

TEST_CASE("EXStar fixture frames reproduce EXStar's own mesh") {
    const auto m = test_data::mustang();
    if (!m) SKIP("mustang fixture not available (tests/fixtures/external/README.md)");
    REQUIRE(m->frame_count == 6055);
    CHECK(std::abs(m->project->baseline_mm() - 159.84) < 0.1);

    auto first = m->read(0);
    REQUIRE(first.has_value());
    CHECK(first->id == 1);
    CHECK(first->T_world_camera.matrix().isIdentity(1e-9));
    CHECK(first->depth.width() == 640);
    CHECK(first->depth.height() == 512);

    auto mesh = fixtures::load_stl(m->stl);
    REQUIRE(mesh.has_value());
    const fixtures::MeshDistance dist(*mesh);
    std::vector<float> errs;
    for (const std::size_t idx : fixtures::mustang_mesh_frames()) {
        auto f = m->read(idx);
        REQUIRE(f.has_value());
        const Eigen::Matrix4f T = f->T_world_camera.matrix().cast<float>();
        for (const auto& p : fixtures::unproject(*f, 8)) {
            const Vec3f w = (T * p.homogeneous()).head<3>();
            if (auto d = dist.distance(w, 5.0f)) errs.push_back(*d);
        }
    }
    REQUIRE(errs.size() > 1000);
    std::ranges::sort(errs);
    double ss = 0;
    for (float e : errs) ss += e * e;
    const double median = errs[errs.size() / 2], rms = std::sqrt(ss / static_cast<double>(errs.size()));
    std::println("fixture vs STL: {} pts, median {:.3f} mm, rms {:.3f} mm", errs.size(), median, rms);
    CHECK(median < 0.2);
    CHECK(rms < 0.4);
}
