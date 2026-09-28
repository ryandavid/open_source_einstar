#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <print>

#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/track/tracker.hpp"

using namespace einstar;

// Diagnostic: does a frame placed at the tracker's (diverged) pose still fit EXStar's final STL?
// If so the region is geometrically ambiguous (surface of revolution), not a tracker bug.
TEST_CASE("symmetry probe on mustang frames 480-520", "[.probe]") {
    const auto dir = std::filesystem::path(std::getenv("HOME")) / "Documents/EXStar/mustang_differential";
    if (!std::filesystem::exists(dir / "Project1.data_base")) SKIP("fixture not available");
    auto proj = fixtures::ExstarProject::open(dir / "Project1.ir_E10_prj");
    auto mesh = fixtures::load_stl(dir / "mustang_differential_simplified.stl");
    REQUIRE((proj && mesh));
    const fixtures::MeshDistance dist(*mesh);
    track::Tracker tracker;
    auto fit = [&](const fixtures::Frame& f, const SE3& pose) {
        std::vector<float> e;
        int miss = 0;
        const Eigen::Matrix4f T = pose.matrix().cast<float>();
        for (const auto& p : fixtures::unproject(f, 6)) {
            if (auto d = dist.distance((T * p.homogeneous()).head<3>(), 3.0f)) e.push_back(*d);
            else ++miss;
        }
        std::ranges::sort(e);
        return std::pair{e.empty() ? -1.0f : e[e.size() / 2], static_cast<double>(miss) / static_cast<double>(e.size() + miss)};
    };
    for (std::size_t i = 0; i <= 520; ++i) {
        auto f = (*proj)->read_frame(i);
        REQUIRE(f.has_value());
        const track::Intrinsics k{640, 512, f->intrinsics.fx, f->intrinsics.fy, f->intrinsics.cx, f->intrinsics.cy};
        if (i == 0) tracker.set_initial_pose(f->T_world_camera);
        auto df = track::make_depth_frame(f->depth, k);
        df.timestamp_s = static_cast<double>(i) * 0.068;
        const auto r = tracker.process(df);
        if (i >= 500 && i % 5 == 0) {
            const auto [m_ex, miss_ex] = fit(*f, f->T_world_camera);
            const auto [m_us, miss_us] = fit(*f, r.T_world_camera);
            std::println("frame {}: EXStar pose median {:.3f} mm ({:.0f}% >3mm) | ours median {:.3f} mm ({:.0f}% >3mm), err {:.1f} mm",
                         i, m_ex, 100 * miss_ex, m_us, 100 * miss_us,
                         translation_norm(f->T_world_camera.inverse() * r.T_world_camera));
        }
    }
}
