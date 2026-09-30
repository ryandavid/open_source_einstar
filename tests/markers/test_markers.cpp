#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <print>
#include <format>
#include <random>

#include <Eigen/Dense>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/core/timing.hpp"
#include "real_data.hpp"
#include "einstar/markers/detect.hpp"
#include "einstar/markers/stereo.hpp"

using namespace einstar;

namespace {

// Minimal 8-bit BMP reader (EXStar's calibration captures).
ImageU8 read_bmp8(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(f)), {});
    if (b.size() < 54) return {};
    auto u32 = [&](std::size_t o) { return static_cast<std::uint32_t>(b[o] | b[o + 1] << 8 | b[o + 2] << 16 | b[o + 3] << 24); };
    const auto off = u32(10);
    const auto w = static_cast<int>(u32(18));
    const auto hs = static_cast<std::int32_t>(u32(22));
    const int h = std::abs(hs);
    const int stride = (w + 3) & ~3;
    ImageU8 img(w, h);
    for (int y = 0; y < h; ++y) {
        const int src_row = hs > 0 ? h - 1 - y : y;  // bottom-up by default
        std::copy_n(b.begin() + off + static_cast<std::ptrdiff_t>(src_row) * stride, w, img.data() + static_cast<std::ptrdiff_t>(y) * w);
    }
    return img;
}

const char* kCalCache = EINSTAR_TEST_CALIBRATION_DIR;

}  // namespace

TEST_CASE("ellipse fit recovers a synthetic ellipse") {
    std::vector<Vec2> pts;
    std::mt19937 rng(1);
    std::normal_distribution<double> n(0, 0.02);
    for (int i = 0; i < 60; ++i) {
        const double t = 2 * M_PI * i / 60;
        const double u = 7.0 * std::cos(t), v = 4.0 * std::sin(t), ang = 0.6;
        pts.emplace_back(100.3 + std::cos(ang) * u - std::sin(ang) * v + n(rng), 50.7 + std::sin(ang) * u + std::cos(ang) * v + n(rng));
    }
    markers::Ellipse e;
    REQUIRE(markers::fit_ellipse(pts, e));
    CHECK(std::abs(e.center.x() - 100.3) < 0.01);
    CHECK(std::abs(e.center.y() - 50.7) < 0.01);
    CHECK(std::abs(e.a - 7.0) < 0.02);
    CHECK(std::abs(e.b - 4.0) < 0.02);
    CHECK(e.residual < 0.05);
}

TEST_CASE("markers on the real calibration board triangulate to the board pitch") {
    const auto board = test_data::calibration_board();
    if (!board) SKIP("calibration captures not available (tests/fixtures/external/README.md)");
    const std::string kCalImages = board->string();
    auto cal = calib::load_ccf_directory(kCalCache);
    REQUIRE(cal.has_value());
    const auto rig = cal->rig();
    const auto rect = calib::compute_rectification(rig);
    markers::StereoParams sp;
    sp.diameters.clear();  // the board dots are not scan markers; accept any size
    const markers::MarkerStereo stereo(rig, rect, sp);

    std::vector<double> spacing, plane_rms;
    int images = 0, total_markers = 0;
    double detect_ms = 0;
    for (int k = 1; k <= 25; ++k) {
        const auto l = read_bmp8(std::format("{}/imageLeft{}.bmp", kCalImages, k));
        const auto r = read_bmp8(std::format("{}/imageRight{}.bmp", kCalImages, k));
        if (l.empty() || r.empty()) continue;
        Stopwatch sw;
        const auto el = markers::detect_markers(l.view());
        const auto er = markers::detect_markers(r.view());
        detect_ms += sw.elapsed_ms();
        // A disparity prior from the board's four large dots (unambiguous by size) stands in for the
        // dense speckle depth the live pipeline provides: fit a plane in disparity space.
        const auto big = [](const std::vector<markers::Ellipse>& es) {
            std::vector<markers::Ellipse> out;
            std::vector<double> sizes;
            for (const auto& e : es) sizes.push_back(e.a);
            std::ranges::sort(sizes);
            const double med = sizes.empty() ? 0 : sizes[sizes.size() / 2];
            for (const auto& e : es)
                if (e.a > 1.6 * med) out.push_back(e);
            return out;
        };
        const auto seeds = stereo.reconstruct(big(el), big(er));
        markers::MarkerStereo::DisparityPrior prior;
        if (seeds.size() >= 3) {
            Eigen::MatrixXd A(seeds.size(), 3);
            Eigen::VectorXd bvec(seeds.size());
            for (std::size_t s2 = 0; s2 < seeds.size(); ++s2) {
                A.row(static_cast<Eigen::Index>(s2)) << seeds[s2].left_rect.x(), seeds[s2].left_rect.y(), 1.0;
                bvec(static_cast<Eigen::Index>(s2)) = seeds[s2].left_rect.x() - seeds[s2].right_rect.x();
            }
            const Eigen::Vector3d plane = A.colPivHouseholderQr().solve(bvec);
            prior = [plane](const Vec2& q, double) { return plane(0) * q.x() + plane(1) * q.y() + plane(2); };
        }
        const auto m = stereo.reconstruct(el, er, prior);
        if (m.size() < 20) continue;
        ++images;
        total_markers += static_cast<int>(m.size());
        // Nearest-neighbour spacing (grid pitch) and planarity.
        Vec3 c = Vec3::Zero();
        for (const auto& a : m) c += a.position;
        c /= static_cast<double>(m.size());
        Mat3 cov = Mat3::Zero();
        for (const auto& a : m) cov += (a.position - c) * (a.position - c).transpose();
        Eigen::SelfAdjointEigenSolver<Mat3> es(cov);
        const Vec3 nrm = es.eigenvectors().col(0);
        double ss = 0;
        for (const auto& a : m) ss += std::pow(nrm.dot(a.position - c), 2);
        plane_rms.push_back(std::sqrt(ss / static_cast<double>(m.size())));
        for (std::size_t i = 0; i < m.size(); ++i) {
            double best = 1e9;
            for (std::size_t j = 0; j < m.size(); ++j)
                if (i != j) best = std::min(best, (m[i].position - m[j].position).norm());
            if (best > 20 && best < 36) spacing.push_back(best);
        }
    }
    REQUIRE(images >= 10);
    std::ranges::sort(spacing);
    std::ranges::sort(plane_rms);
    const double med = spacing[spacing.size() / 2];
    double mad = 0;
    for (double s : spacing) mad += std::abs(s - med);
    mad /= static_cast<double>(spacing.size());
    std::println("calibration board: {} image pairs, {:.1f} markers/pair, detect {:.1f} ms/pair; pitch median {:.3f} mm (mean abs dev {:.3f}); plane rms median {:.3f} mm",
                 images, static_cast<double>(total_markers) / images, detect_ms / 25, med, mad, plane_rms[plane_rms.size() / 2]);
    // EXStar's own calibration analysis measured 27.89 +/- 0.08 mm on these images.
    CHECK(std::abs(med - 27.9) < 0.15);
    CHECK(mad < 0.15);
    CHECK(plane_rms[plane_rms.size() / 2] < 0.15);
}

#include "einstar/markers/marker_map.hpp"

TEST_CASE("marker map tracks with a pose guess and relocalises without one") {
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(-120, 120);
    std::normal_distribution<double> noise(0, 0.05);
    std::vector<markers::MapMarker> world;
    for (int i = 0; i < 40; ++i) world.push_back({i, Vec3(u(rng), u(rng) * 0.5, 300 + u(rng) * 0.2), 6.0, 10, false});
    markers::MarkerMap map;
    map.set_markers(world);

    Vec6 xi;
    xi << 30, -10, 15, 0.2, -0.1, 0.3;
    const SE3 T_wc = se3_exp(xi);
    // Frame sees 12 markers (plus noise) and 2 spurious detections.
    std::vector<Vec3> frame;
    for (int i = 5; i < 17; ++i) {
        Vec3 p = T_wc.inverse() * world[static_cast<std::size_t>(i)].position;
        p += Vec3(noise(rng), noise(rng), noise(rng));
        frame.push_back(p);
    }
    frame.push_back(Vec3(10, 10, 250));
    frame.push_back(Vec3(-40, 5, 280));

    Vec6 d;
    d << 1.5, -1.0, 1.0, 0.004, 0.003, -0.002;
    auto tr = map.track(frame, se3_exp(d) * T_wc);
    REQUIRE(tr.has_value());
    CHECK(tr->inliers.size() == 12);
    CHECK(translation_norm(T_wc.inverse() * tr->T_world_camera) < 0.1);

    auto rl = map.relocalize(frame);
    REQUIRE(rl.has_value());
    CHECK(rl->inliers.size() == 12);
    CHECK(translation_norm(T_wc.inverse() * rl->T_world_camera) < 0.1);
    CHECK(rotation_angle(T_wc.inverse() * rl->T_world_camera) * 180 / M_PI < 0.05);
}
