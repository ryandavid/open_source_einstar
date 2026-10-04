// Model-consistency rejection of depth pixels: what a frame shows is compared with a consensus
// model seen from the frame's camera.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <functional>
#include <random>

#include "einstar/recon/consistency.hpp"

using namespace einstar;

namespace {

// A square plane z = z0 (|x|, |y| <= half), facing -z (toward the origin) or +z.
recon::TriangleMesh plane(float z0, float half, float step, bool facing_origin) {
    recon::TriangleMesh m;
    const int n = static_cast<int>(std::lround(2 * half / step)) + 1;
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) m.vertices.emplace_back(-half + static_cast<float>(i) * step, -half + static_cast<float>(j) * step, z0);
    auto id = [&](int i, int j) { return static_cast<std::uint32_t>(j * n + i); };
    for (int j = 0; j + 1 < n; ++j)
        for (int i = 0; i + 1 < n; ++i) {
            // (i,j) (i+1,j) (i,j+1) winds to +z.
            if (facing_origin) {
                m.triangles.push_back({id(i, j), id(i, j + 1), id(i + 1, j)});
                m.triangles.push_back({id(i + 1, j), id(i, j + 1), id(i + 1, j + 1)});
            } else {
                m.triangles.push_back({id(i, j), id(i + 1, j), id(i, j + 1)});
                m.triangles.push_back({id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)});
            }
        }
    m.compute_normals();
    return m;
}

recon::TriangleMesh merged(const recon::TriangleMesh& a, const recon::TriangleMesh& b) {
    recon::TriangleMesh m = a;
    const auto off = static_cast<std::uint32_t>(a.vertices.size());
    m.vertices.insert(m.vertices.end(), b.vertices.begin(), b.vertices.end());
    m.normals.insert(m.normals.end(), b.normals.begin(), b.normals.end());
    for (auto t : b.triangles) m.triangles.push_back({t[0] + off, t[1] + off, t[2] + off});
    return m;
}

const track::Intrinsics kCam{160, 128, 200.0, 200.0, 80.0, 64.0};  // sees +-120 x +-96 mm at 300 mm

// Depth of the plane z = z0 (world) seen from a camera at `T_wc` looking along its +z, with noise,
// then `edit(x, y, depth)` per pixel.
ImageF32 plane_depth(double z0, const SE3& T_wc, const std::function<void(int, int, float&)>& edit = {}) {
    ImageF32 d(kCam.width, kCam.height, 0.0f);
    std::mt19937 rng(3);
    std::normal_distribution<float> noise(0.0f, 0.03f);
    const Vec3 c = T_wc.translation();
    for (int y = 0; y < kCam.height; ++y)
        for (int x = 0; x < kCam.width; ++x) {
            const Vec3 r = T_wc.linear() * Vec3((x - kCam.cx) / kCam.fx, (y - kCam.cy) / kCam.fy, 1.0);
            const double t = (z0 - c.z()) / r.z();  // camera depth: r has unit camera z
            if (t <= 0) continue;
            d(x, y) = static_cast<float>(t) + noise(rng);
            if (edit) edit(x, y, d(x, y));
        }
    return d;
}

bool in(int x, int y, int x0, int y0, int x1, int y1) { return x >= x0 && x < x1 && y >= y0 && y < y1; }

}  // namespace

TEST_CASE("consistency: floaters and hidden pixels are dropped, the surface is kept") {
    const auto model_mesh = plane(300.0f, 125.0f, 1.25f, true);
    const recon::ConsistencyModel model(model_mesh);
    const SE3 T = SE3::Identity();
    auto frame = track::make_depth_frame(plane_depth(300.0, T,
                                                     [](int x, int y, float& z) {
                                                         if (in(x, y, 20, 20, 40, 40)) z = 280.0f;  // a flake 20 mm in front
                                                         if (in(x, y, 100, 20, 120, 40)) z += 3.0f;  // 3 mm behind
                                                         if (in(x, y, 60, 80, 80, 100)) z += 0.3f;  // within the noise
                                                     }),
                                         kCam);
    const recon::ConsistencyParams params;
    CHECK(params.tolerance(300.0f) < 1.0f);
    const auto st = model.filter(frame, T, params);
    int flake_left = 0, behind_left = 0, near_kept = 0, rest_dropped = 0, rest = 0;
    for (int y = 0; y < kCam.height; ++y)
        for (int x = 0; x < kCam.width; ++x) {
            const bool valid = frame.points(x, y).z() > 0;
            if (in(x, y, 20, 20, 40, 40)) flake_left += valid;
            else if (in(x, y, 100, 20, 120, 40)) behind_left += valid;
            else if (in(x, y, 60, 80, 80, 100)) near_kept += valid;
            else {
                ++rest;
                rest_dropped += !valid;
            }
        }
    CHECK(flake_left == 0);
    CHECK(behind_left == 0);
    CHECK(near_kept == 400);
    CHECK(rest_dropped < rest / 200);  // only along the edges of the edited patches (no normal there)
    CHECK(st.in_front >= 400);
    CHECK(st.behind >= 400);
    CHECK(st.unseen == 0);
}

TEST_CASE("consistency: the far face of a thin part is kept") {
    // A 2 mm sheet at z = 300..302. The camera at the origin sees its near face; one at z = 600,
    // looking back, its far face.
    SE3 behind = SE3::Identity();
    behind.linear() = Eigen::AngleAxisd(M_PI, Vec3::UnitY()).toRotationMatrix();
    behind.translation() = Vec3(0, 0, 600);
    const recon::ConsistencyParams params;
    SECTION("the model has only the near face: the far camera sees its back, which says nothing") {
        const auto mesh = plane(300.0f, 125.0f, 1.25f, true);
        const recon::ConsistencyModel model(mesh);
        auto far = track::make_depth_frame(plane_depth(302.0, behind), kCam);
        const auto st = model.filter(far, behind, params);
        CHECK(st.pixels > 15000);
        CHECK(st.rejected() == 0);
        CHECK(st.unseen == st.pixels);
    }
    SECTION("the model has both faces: each camera agrees with its own") {
        const auto mesh = merged(plane(300.0f, 125.0f, 1.25f, true), plane(302.0f, 125.0f, 1.25f, false));
        const recon::ConsistencyModel model(mesh);
        auto near = track::make_depth_frame(plane_depth(300.0, SE3::Identity()), kCam);
        auto far = track::make_depth_frame(plane_depth(302.0, behind), kCam);
        const auto a = model.filter(near, SE3::Identity(), params);
        const auto b = model.filter(far, behind, params);
        CHECK(a.rejected() == 0);
        CHECK(b.rejected() == 0);
        CHECK(a.unseen == 0);
        CHECK(b.unseen == 0);
    }
}

TEST_CASE("consistency: a surface behind a flake of the model is kept") {
    // The model has a 20 x 20 mm flake 20 mm in front of the plane; the frame sees the plane
    // through it (the flake is not there). The plane supports those pixels.
    auto flake = plane(280.0f, 10.0f, 1.0f, true);
    const auto mesh = merged(plane(300.0f, 125.0f, 1.25f, true), flake);
    const recon::ConsistencyModel model(mesh);
    auto frame = track::make_depth_frame(plane_depth(300.0, SE3::Identity()), kCam);
    const auto st = model.filter(frame, SE3::Identity(), recon::ConsistencyParams{});
    CHECK(st.pixels > 15000);
    CHECK(st.rejected() == 0);
}

TEST_CASE("consistency: the model's depth seen from a camera") {
    const auto mesh = plane(300.0f, 200.0f, 2.0f, true);
    const recon::ConsistencyModel model(mesh);
    const auto view = model.render(SE3::Identity(), kCam);
    int holes = 0;
    double worst = 0;
    for (int y = 0; y < kCam.height; ++y)
        for (int x = 0; x < kCam.width; ++x) {
            if (view.depth(x, y) <= 0) {
                ++holes;
                continue;
            }
            worst = std::max(worst, std::abs(view.depth(x, y) - 300.0));
            CHECK(view.front_facing(x, y, kCam));
        }
    CHECK(holes == 0);
    CHECK(worst < 1e-3);
}
