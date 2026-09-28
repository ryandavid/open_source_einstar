// Prints live marker reconstruction on synthetic views against ground truth (debugging aid).
#include <cstdio>
#include <cstdlib>
#include <format>
#include <print>
#include <random>
#include "einstar/pipeline/stereo_frontend.hpp"
#include "synthetic_setup.hpp"
using namespace einstar;

int main(int argc, char** argv) {
    const auto rig = e2e::einstar_like_rig();
    auto setup = e2e::make_scene();
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> ux(-260, 120), uz(-120, 170);
    for (int tries = 0; tries < 20000 && setup.scene.markers.size() < 45; ++tries) {
        const Vec3 c(ux(rng), 70.0, uz(rng));
        bool ok = true;
        for (const auto& m : setup.scene.markers) ok = ok && (m.center - c).norm() > 24.0;
        if (ok) setup.scene.markers.push_back({c, Vec3(0, -1, 0), 6.0, 10.0});
    }
    pipeline::StereoFrontendParams fp;
    fp.marker_stereo.require_prior = std::getenv("NOPRIOR") == nullptr;
    pipeline::StereoFrontend fe(rig, fp);
    const SE3 T_left_rect = fe.rectification().T_left_rectified();
    for (int a = 1; a < argc; ++a) {
        const auto id = static_cast<std::uint32_t>(std::atoi(argv[a]));
        ImageU8 l, r;
        e2e::render_sensor(setup, rig, e2e::truth_pose(id), 0, id * 3, l);
        e2e::render_sensor(setup, rig, e2e::truth_pose(id), 1, id * 3 + 1, r);
        const auto el = markers::detect_markers(l.view()), er = markers::detect_markers(r.view());
        auto out = fe.process(l, r);
        if (const char* dir = std::getenv("DUMP_DIR")) {  // writes <dir>/left_<id>.pgm
            std::FILE* f = std::fopen(std::format("{}/left_{}.pgm", dir, id).c_str(), "wb");
            std::fprintf(f, "P5 %d %d 255\n", l.width(), l.height());
            std::fwrite(l.data(), 1, static_cast<std::size_t>(l.width() * l.height()), f);
            std::fclose(f);
        }
        const SE3 T = e2e::truth_pose(id) * T_left_rect;
        // Which truth markers are visible (in front, inside the left image; occlusion ignored)?
        const SE3 Tcw = e2e::truth_pose(id).inverse();
        for (const auto& mk : setup.scene.markers) {
            const Vec3 pc = Tcw * mk.center;
            if (pc.z() <= 0) continue;
            const double u = rig.left.fx * pc.x() / pc.z() + rig.left.cx, v = rig.left.fy * pc.y() / pc.z() + rig.left.cy;
            if (u < 0 || v < 0 || u >= 1280 || v >= 1024) continue;
            double best = 1e9;
            for (const auto& e : el) best = std::min(best, (e.center - Vec2(u, v)).norm());
            const Vec3 ray = pc.normalized();
            std::println("   truth marker z {:6.1f} px ({:6.1f},{:6.1f}) incidence {:4.1f} deg, radius ~{:.1f} px: nearest left detection {:.1f} px",
                         pc.z(), u, v, std::acos(std::abs(ray.dot(Tcw.linear() * mk.normal))) * 180 / M_PI, 3.0 * rig.left.fx / pc.z(), best);
        }
        std::println("frame {}: {} left / {} right detections, {} stereo markers, {} unmatched left", id, el.size(), er.size(),
                     out.markers.size(), out.unmatched_left.size());
        const auto* dev = out.frame.device.get();
        int W = dev ? dev->width() : out.frame.points.width();
        int valid_total = 0;
        if (dev)
            for (int k = 0; k < dev->width() * dev->height(); ++k) valid_total += dev->points_xyzw()[4 * k + 2] > 0;
        std::println("  depth frame {}x{} device {} valid {}", W, dev ? dev->height() : out.frame.points.height(), dev != nullptr, valid_total);
        for (const auto& m : out.markers) {
            const Vec3 w = T * m.position;
            if (dev) {
                const int x = static_cast<int>(m.left_rect.x() / 2), y = static_cast<int>(m.left_rect.y() / 2);
                int n = 0;
                double zs = 0;
                for (int yy = y - 12; yy <= y + 12; ++yy)
                    for (int xx = x - 12; xx <= x + 12; ++xx) {
                        if (xx < 0 || yy < 0 || xx >= W || yy >= dev->height()) continue;
                        const float z = dev->points_xyzw()[4 * (yy * W + xx) + 2];
                        if (z > 0) { ++n; zs += z; }
                    }
                std::println("    window valid {} mean z {:.1f}", n, n ? zs / n : 0.0);
            }
            double best = 1e9;
            for (const auto& s : setup.scene.markers) best = std::min(best, (s.center - w).norm());
            std::println("  L({:6.1f},{:6.1f}) R({:6.1f},{:6.1f}) z {:6.1f} diam {:.2f} truth err {:.2f} mm", m.left_rect.x(), m.left_rect.y(),
                         m.right_rect.x(), m.right_rect.y(), m.position.z(), m.diameter, best);
        }
    }
}
