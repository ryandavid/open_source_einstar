#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>

#include "einstar/track/relocaliser.hpp"

using namespace einstar;
using track::GlobalRelocaliser;

namespace {

// An irregular set of boxes and a sphere sampled at 1 mm: distinctive enough for descriptors.
std::vector<track::SurfacePoint> scene_points() {
    std::vector<track::SurfacePoint> pts;
    auto box = [&](Vec3f lo, Vec3f hi) {
        for (int axis = 0; axis < 3; ++axis)
            for (int side = 0; side < 2; ++side) {
                const int a = (axis + 1) % 3, b = (axis + 2) % 3;
                for (float u = lo[a]; u <= hi[a]; u += 1.0f)
                    for (float v = lo[b]; v <= hi[b]; v += 1.0f) {
                        Vec3f p;
                        p[axis] = side ? hi[axis] : lo[axis];
                        p[a] = u;
                        p[b] = v;
                        Vec3f n = Vec3f::Zero();
                        n[axis] = side ? 1.0f : -1.0f;
                        pts.push_back({p, n, 1.0f});
                    }
            }
    };
    box({0, 0, 0}, {60, 20, 30});
    box({70, -10, 5}, {95, 30, 18});
    box({10, 25, 0}, {30, 55, 45});
    const Vec3f c(50, 45, 20);
    for (float th = 0; th < static_cast<float>(M_PI); th += 0.04f)
        for (float ph = 0; ph < 2 * static_cast<float>(M_PI); ph += 0.04f) {
            const Vec3f n(std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th));
            pts.push_back({c + 15.0f * n, n, 1.0f});
        }
    std::ranges::shuffle(pts, std::mt19937(3));  // GPU extraction order is arbitrary too
    return pts;
}

track::OrientedCloud view_of(const std::vector<track::SurfacePoint>& pts, const SE3& T_model_frame) {
    // The frame sees the scene from its own coordinates.
    const SE3 T_frame_model = T_model_frame.inverse();
    track::OrientedCloud c;
    for (const auto& p : pts) {
        c.points.push_back((T_frame_model.cast<float>() * p.position).eval());
        c.normals.push_back(T_frame_model.linear().cast<float>() * p.normal);
    }
    return c;
}

}  // namespace

TEST_CASE("relocaliser: deterministic results arrive exactly when due") {
    GlobalRelocaliser rl({}, true);
    double wait = 0;
    rl.submit_model(scene_points(), 5);
    REQUIRE(rl.model_pending());
    CHECK(rl.model(4, wait) == nullptr);  // not due yet, even if the worker has finished
    const auto model = rl.model(5, wait);
    REQUIRE(model != nullptr);
    CHECK_FALSE(rl.model_pending());

    SE3 T = SE3::Identity();
    T.linear() = Eigen::AngleAxisd(0.7, Vec3(0.2, 1, 0.3).normalized()).toRotationMatrix();
    T.translation() = Vec3(40, -25, 60);
    rl.submit_query(model, view_of(scene_points(), T), 1, 12);
    CHECK_FALSE(rl.take_result(11, wait).has_value());
    CHECK(rl.query_pending());
    const auto res = rl.take_result(12, wait);
    REQUIRE(res.has_value());
    REQUIRE(res->has_value());
    const SE3 err = T.inverse() * (*res)->T_model_frame;
    CHECK(translation_norm(err) < 2.0);
    CHECK(rotation_angle(err) < 0.02);
    CHECK_FALSE(rl.query_pending());
}

TEST_CASE("relocaliser: clear drops outstanding work") {
    GlobalRelocaliser rl({}, false);
    double wait = 0;
    rl.submit_model(scene_points(), 0);
    rl.clear();
    CHECK_FALSE(rl.model_pending());
    CHECK(rl.model(100, wait) == nullptr);
    CHECK_FALSE(rl.take_result(100, wait).has_value());
    // Still usable afterwards; live mode adopts a finished model without a due frame.
    rl.submit_model(scene_points(), 1000);
    std::shared_ptr<const track::FeatureModel> m;
    for (int i = 0; i < 2000 && !m; ++i) {
        m = rl.model(0, wait);
        if (!m) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(m != nullptr);
    CHECK(wait == 0);  // live mode never waits
}
