// Process step on a synthetic session: exact depth of an analytic scene along a closed loop, with
// drift injected into the "live" poses. Loop closure must remove the drift, and the mesh must match
// the scene.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <print>
#include <random>

#include "einstar/recon/process.hpp"
#include "einstar/session/session.hpp"
#include "einstar/synth/speckle_scene.hpp"

using namespace einstar;

namespace {

SE3 look_at(const Vec3& eye, const Vec3& target) {
    const Vec3 f = (target - eye).normalized();
    const Vec3 r = Vec3(0, 1, 0).cross(f).normalized() * -1.0;
    SE3 T = SE3::Identity();
    T.linear().col(0) = r;
    T.linear().col(1) = f.cross(r);
    T.linear().col(2) = f;
    T.translation() = eye;
    return T;
}

// Closed orbit above the table, looking at the objects.
SE3 truth_pose(int i, int n) {
    const double a = 2 * M_PI * i / n;
    const Vec3 center(-60, 40, 0);
    const Vec3 eye = center + Vec3(170 * std::cos(a), -230, 170 * std::sin(a));
    return look_at(eye, center + Vec3(20 * std::cos(a), 0, 20 * std::sin(a)));
}

synth::Scene make_scene() {
    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    auto box = [&](Vec3 c, Vec3 half, double yaw_deg) {
        SE3 T = SE3::Identity();
        T.linear() = Eigen::AngleAxisd(yaw_deg * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
        T.translation() = c;
        scene.primitives.push_back(synth::Box{T, half});
    };
    scene.primitives.push_back(synth::Sphere{Vec3(-80, 25, 20), 45.0});
    box(Vec3(-10, 45, 40), Vec3(25, 25, 18), 30);
    box(Vec3(-150, 50, -10), Vec3(20, 20, 30), -20);
    box(Vec3(-60, 55, -70), Vec3(35, 15, 12), 55);
    box(Vec3(-120, 40, 70), Vec3(12, 30, 12), 10);
    return scene;
}

CameraModel small_camera() {
    CameraModel cam;
    cam.width = 320;
    cam.height = 256;
    cam.fx = cam.fy = 290;
    cam.cx = 160;
    cam.cy = 128;
    return cam;
}

// Writes a session of exact (noisy) depth rendered at `truth`, recorded with the given live poses.
std::string write_session(const std::string& name, const synth::Scene& scene, const std::vector<SE3>& truth, const std::vector<SE3>& live,
                          const std::vector<std::uint32_t>& flags) {
    const CameraModel cam = small_camera();
    synth::Projector proj;  // depth only: the projector pattern is irrelevant
    proj.model = cam;
    proj.pattern = synth::DotPattern::random(64, 64, 10, 2.0, 1);
    const auto path = (std::filesystem::temp_directory_path() / name).string();
    session::SessionHeader h;
    h.depth_intrinsics = {cam.width, cam.height, cam.fx, cam.fy, cam.cx, cam.cy};
    auto w = session::SessionWriter::create(path, h);
    REQUIRE(w.has_value());
    std::mt19937 rng(9);
    std::normal_distribution<float> noise(0.0f, 0.04f);
    for (std::size_t i = 0; i < truth.size(); ++i) {
        synth::RenderParams rp;
        rp.supersample = 1;
        const auto view = synth::render_view(scene, proj, cam, truth[i], rp);
        session::FrameRecord f;
        f.index = i;
        f.timestamp_s = 0.068 * static_cast<double>(i);
        f.flags = flags[i];
        f.T_world_camera = live[i];
        f.depth = view.depth;
        for (auto& z : f.depth.pixels())
            if (z > 0) z += noise(rng);
        (*w)->write(std::move(f));
    }
    (*w)->close();
    return path;
}

}  // namespace

TEST_CASE("process: loop closure removes drift and the mesh matches the scene") {
    const int n = 72;
    const auto scene = make_scene();
    // Live poses drift: a small rotation and translation per frame, accumulated.
    SE3 drift_step = SE3::Identity();
    // ~0.02 deg and ~0.03 mm per frame, all in one direction: 1.4 deg / 3.5 mm around the loop (worse
    // than the live tracker's typical random frame-to-frame error).
    const double drift_scale = 0.33;
    drift_step.linear() = Eigen::AngleAxisd(drift_scale * 0.06 * M_PI / 180, Vec3(0.3, 1, 0.2).normalized()).toRotationMatrix();
    drift_step.translation() = drift_scale * Vec3(0.08, -0.05, 0.06);

    std::vector<SE3> truth(n), live(n);
    SE3 drift = SE3::Identity();
    for (int i = 0; i < n; ++i) {
        truth[static_cast<std::size_t>(i)] = truth_pose(i, n);
        live[static_cast<std::size_t>(i)] = drift * truth[static_cast<std::size_t>(i)];
        drift = drift_step * drift;
    }
    const auto path = write_session("einstar_process_test.estr", scene, truth, live,
                                    std::vector<std::uint32_t>(static_cast<std::size_t>(n), session::frame_accepted | session::frame_integrated));
    auto s = session::SessionReader::open(path);
    REQUIRE(s.has_value());

    auto pose_errors = [&](auto get) {
        double mt = 0, mr = 0;
        for (int i = 0; i < n; ++i) {
            const SE3 e = truth[static_cast<std::size_t>(i)].inverse() * get(i);
            mt = std::max(mt, translation_norm(e));
            mr = std::max(mr, rotation_angle(e) * 180 / M_PI);
        }
        return std::pair{mt, mr};
    };
    const auto [live_t, live_r] = pose_errors([&](int i) { return live[static_cast<std::size_t>(i)]; });

    recon::ProcessParams pp;
    pp.fragment_frames = 6;
    pp.simplify = false;  // vertex-sampled accuracy below is biased by decimation (tested separately)
    pp.use_gpu = true;
    auto r = recon::process_session(**s, pp);
    REQUIRE(r.has_value());
    const auto& rep = r->report;
    REQUIRE(r->frame_poses.size() == static_cast<std::size_t>(n));
    const auto [opt_t, opt_r] = pose_errors([&](int i) { return r->frame_poses.at(static_cast<std::size_t>(i)); });

    // Mesh accuracy against the analytic scene (distance along the surface normal via ray casts).
    double sum = 0, mx = 0;
    int cnt = 0, far = 0;
    for (std::size_t v = 0; v < r->mesh.vertices.size(); v += 7) {
        const Vec3 p = r->mesh.vertices[v].cast<double>();
        const Vec3 nrm = r->mesh.normals[v].cast<double>();
        // Closest scene hit along +-normal.
        double best = 1e9;
        for (const double sgn : {1.0, -1.0}) {
            const auto hit = scene.intersect(p - sgn * 2.0 * nrm, sgn * nrm);
            if (hit) best = std::min(best, std::abs(hit->t - 2.0));
        }
        if (best > 1.0) {
            ++far;
            continue;
        }
        sum += best;
        mx = std::max(mx, best);
        ++cnt;
    }
    std::println("process: {} frames, {} fragments, {} odometry + {} loop edges ({} candidates, {} pruned); "
                 "pose error live {:.2f} mm {:.2f} deg -> optimised {:.2f} mm {:.3f} deg; mesh {} vertices, {} triangles, "
                 "error mean {:.3f} max {:.3f} mm ({} of {} samples off-surface); stages {:.0f}/{:.0f}/{:.0f}/{:.0f} ms",
                 rep.frames_used, rep.fragments, rep.odometry_edges, rep.loop_edges, rep.loop_candidates, rep.loop_edges_pruned, live_t,
                 live_r, opt_t, opt_r, rep.vertices, rep.triangles, cnt ? sum / cnt : 0.0, mx, far, cnt + far,
                 rep.stage_ms.at("fragments"), rep.stage_ms.at("pose graph"), rep.stage_ms.at("fusion"), rep.stage_ms.at("mesh"));
    CHECK(live_t > 3.0);  // the test is meaningful
    CHECK(rep.loop_edges > 0);
    CHECK(opt_t < 0.5);
    CHECK(opt_r < 0.15);
    CHECK(rep.triangles > 10000);
    CHECK(sum / cnt < 0.08);
    CHECK(far < (cnt + far) / 100);
    std::filesystem::remove(path);
}

TEST_CASE("process: a segment placed by a wrong relocalisation is left out") {
    // Live tracking "relocalised" frames 36-47 onto a wrong pose (rotated 90 degrees about the scene)
    // and back at 48. Nothing verifies that segment against the rest, and it contradicts the model.
    const int n = 72;
    const auto scene = make_scene();
    std::vector<SE3> truth(n), live(n);
    std::vector<std::uint32_t> flags(n, session::frame_accepted | session::frame_integrated);
    SE3 wrong = SE3::Identity();
    wrong.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
    wrong.translation() = Vec3(-60, 40, 0) - wrong.linear() * Vec3(-60, 40, 0);  // about the scene centre
    for (int i = 0; i < n; ++i) {
        truth[static_cast<std::size_t>(i)] = truth_pose(i, n);
        live[static_cast<std::size_t>(i)] = (i >= 36 && i < 48) ? wrong * truth[static_cast<std::size_t>(i)] : truth[static_cast<std::size_t>(i)];
    }
    flags[36] |= session::frame_relocalized;
    flags[48] |= session::frame_relocalized;
    const auto path = write_session("einstar_island_test.estr", scene, truth, live, flags);
    auto s = session::SessionReader::open(path);
    REQUIRE(s.has_value());
    recon::ProcessParams pp;
    pp.fragment_frames = 6;
    pp.simplify = false;
    pp.recover_lost_frames = false;
    auto r = recon::process_session(**s, pp);
    REQUIRE(r.has_value());
    std::println("island: {} islands, {} excluded, {} frames left out", r->report.islands, r->report.islands_excluded, r->report.frames_excluded);
    CHECK(r->report.islands_excluded == 1);
    CHECK(r->report.frames_excluded == 12);
    for (int i = 36; i < 48; ++i) CHECK(!r->frame_poses.contains(static_cast<std::size_t>(i)));
    std::filesystem::remove(path);
}
