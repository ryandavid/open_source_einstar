#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <print>

#include "einstar/fixtures/exstar_project.hpp"

using namespace einstar;

namespace {
const std::filesystem::path kMustang = std::filesystem::path(std::getenv("HOME")) / "Documents/EXStar/mustang_differential";
}

TEST_CASE("EXStar fixture frames reproduce EXStar's own mesh") {
    if (!std::filesystem::exists(kMustang / "Project1.data_base")) SKIP("mustang fixture not available");
    auto proj = fixtures::ExstarProject::open(kMustang / "Project1.ir_E10_prj");
    REQUIRE(proj.has_value());
    REQUIRE((*proj)->frame_count() == 6055);
    CHECK(std::abs((*proj)->baseline_mm() - 159.84) < 0.1);

    auto first = (*proj)->read_frame(0);
    REQUIRE(first.has_value());
    CHECK(first->id == 1);
    CHECK(first->T_world_camera.matrix().isIdentity(1e-9));
    CHECK(first->depth.width() == 640);
    CHECK(first->depth.height() == 512);

    auto mesh = fixtures::load_stl(kMustang / "mustang_differential_simplified.stl");
    REQUIRE(mesh.has_value());
    const fixtures::MeshDistance dist(*mesh);
    std::vector<float> errs;
    for (const std::size_t idx : {0uz, 1500uz, 3000uz, 4500uz, 6000uz}) {
        auto f = (*proj)->read_frame(idx);
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
