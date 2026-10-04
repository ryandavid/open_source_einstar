// Depth error of the stereo frontend against the synthetic scene, split by distance to a depth
// discontinuity (debugging aid for silhouette outliers).
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <print>
#include "einstar/pipeline/stereo_frontend.hpp"
#include "synthetic_setup.hpp"
using namespace einstar;

int main(int argc, char** argv) {
    const auto rig = e2e::einstar_like_rig();
    const auto setup = e2e::make_scene();
    pipeline::StereoFrontendParams fp_cpu_previews;
    fp_cpu_previews.cpu_previews = true;
    if (const char* ref = std::getenv("REF_DEPTH")) fp_cpu_previews.reference_depth_mm = std::atof(ref);
    pipeline::StereoFrontend fe(rig, fp_cpu_previews);
    long counts[4][3] = {};  // distance band x {total, >0.5 mm, >2 mm}
    long kind[3][3] = {};    // {interior, hole border, depth jump} x {total, >0.5, >2}
    long signed_hist[2] = {}, sil_hist[2] = {}, no_hit = 0;
    long disp_hist[4] = {}, disp_z[4] = {};
    long tex[2][6] = {};  // {good, bad} x texture std bins of 5 grey levels
    for (int a = 1; a < argc; ++a) {
        const auto id = static_cast<std::uint32_t>(std::atoi(argv[a]));
        ImageU8 l, r;
        e2e::render_sensor(setup, rig, e2e::truth_pose(id), 0, id * 3, l);
        e2e::render_sensor(setup, rig, e2e::truth_pose(id), 1, id * 3 + 1, r);
        auto out = fe.process(l, r);
        out.frame.ensure_cpu();
        const SE3 T = e2e::truth_pose(id) * fe.rectification().T_left_rectified();
        const Vec3 cam = T.translation();
        auto& P = out.frame.points;
        const int R = std::getenv("EDGE_R") ? std::atoi(std::getenv("EDGE_R")) : 0;
        if (const char* mt = std::getenv("MEDIAN_MM")) {
            const float base = static_cast<float>(std::atof(mt));
            const float rel = std::getenv("MEDIAN_REL") ? static_cast<float>(std::atof(std::getenv("MEDIAN_REL"))) : 0.004f;
            const int MR = std::getenv("MEDIAN_R") ? std::atoi(std::getenv("MEDIAN_R")) : 2;
            Image<std::uint8_t> drop(P.width(), P.height(), 0);
            std::vector<float> zs;
            for (int y = 0; y < P.height(); ++y)
                for (int x = 0; x < P.width(); ++x) {
                    const float z = P(x, y).z();
                    if (z <= 0) continue;
                    zs.clear();
                    for (int dy = -MR; dy <= MR; ++dy)
                        for (int dx = -MR; dx <= MR; ++dx) {
                            const int u = x + dx, v = y + dy;
                            if (u < 0 || v < 0 || u >= P.width() || v >= P.height()) continue;
                            if (P(u, v).z() > 0) zs.push_back(P(u, v).z());
                        }
                    std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(zs.size() / 2), zs.end());
                    const float med = zs[zs.size() / 2];
                    if (zs.size() < 5 || std::abs(z - med) > std::max(base, rel * z)) drop(x, y) = 1;
                }
            for (int y = 0; y < P.height(); ++y)
                for (int x = 0; x < P.width(); ++x)
                    if (drop(x, y)) P(x, y) = Vec3f::Zero();
        }
        if (R > 0) {
            // Drop pixels within R of a depth jump (simulated filter).
            Image<std::uint8_t> drop(P.width(), P.height(), 0);
            for (int y = 0; y < P.height(); ++y)
                for (int x = 0; x < P.width(); ++x) {
                    const float z = P(x, y).z();
                    if (z <= 0) continue;
                    for (int dy = -R; dy <= R; ++dy)
                        for (int dx = -R; dx <= R; ++dx) {
                            const int u = x + dx, v = y + dy;
                            if (u < 0 || v < 0 || u >= P.width() || v >= P.height()) continue;
                            const float zz = P(u, v).z();
                            if (zz > 0 && std::abs(zz - z) > std::max(3.0f, 0.01f * z)) drop(x, y) = 1;
                        }
                }
            for (int y = 0; y < P.height(); ++y)
                for (int x = 0; x < P.width(); ++x)
                    if (drop(x, y)) P(x, y) = Vec3f::Zero();
        }
        const int W = P.width(), H = P.height();
        // Distance (px) to the nearest depth discontinuity (jump > 3 mm or invalid neighbour).
        auto near_edge = [&](int x, int y) {
            for (int rr = 1; rr <= 3; ++rr)
                for (int dy = -rr; dy <= rr; ++dy)
                    for (int dx = -rr; dx <= rr; ++dx) {
                        if (std::max(std::abs(dx), std::abs(dy)) != rr) continue;
                        const int u = x + dx, v = y + dy;
                        if (u < 0 || v < 0 || u >= W || v >= H) return rr;
                        const float z = P(u, v).z();
                        if (z <= 0 || std::abs(z - P(x, y).z()) > 3.0f) return rr;
                    }
            return 4;
        };
        for (int y = 0; y < H; ++y)
            for (int x = 0; x < W; ++x) {
                const Vec3f& p = P(x, y);
                if (p.z() <= 0) continue;
                const Vec3 w = T * p.cast<double>();
                const Vec3 dir = (w - cam).normalized();
                const auto hit = setup.scene.intersect(cam, dir);
                const double err = hit ? std::abs(hit->t - (w - cam).norm()) : 1e9;
                if (hit) {
                    // Disparity error (half-resolution px): what the matcher actually gets wrong.
                    const double fb = out.frame.intrinsics.fx * fe.rectification().geometry.baseline;
                    const double zt = (T.inverse() * (cam + dir * hit->t)).z();
                    const double de = std::abs(fb / p.z() - fb / zt);
                    disp_hist[de < 0.25 ? 0 : de < 0.5 ? 1 : de < 1.0 ? 2 : 3]++;
                    if (de >= 1.0) disp_z[std::min(3, static_cast<int>(p.z() / 150.0) - 1)]++;
                    // Local texture (std of the rectified left IR in a 7x7 window).
                    const auto& I = out.rectified_left;
                    double s1 = 0, s2 = 0;
                    int n = 0;
                    for (int dy = -3; dy <= 3; ++dy)
                        for (int dx = -3; dx <= 3; ++dx) {
                            const int u = std::clamp(x + dx, 0, I.width() - 1), v = std::clamp(y + dy, 0, I.height() - 1);
                            s1 += I(u, v);
                            s2 += static_cast<double>(I(u, v)) * I(u, v);
                            ++n;
                        }
                    const double sd = std::sqrt(std::max(0.0, s2 / n - (s1 / n) * (s1 / n)));
                    tex[de >= 1.0 ? 1 : 0][std::min(5, static_cast<int>(sd / 5.0))]++;
                }
                if (hit && err > 2.0) {
                    const double se = (w - cam).norm() - hit->t;  // + = estimated too far
                    signed_hist[se < 0 ? 0 : 1]++;
                    // Is the truth near a silhouette? Probe true ranges 3 px away along the rectified rows.
                    bool sil = false;
                    for (int dx : {-6, -3, 3, 6}) {
                        const int u = x + dx;
                        if (u < 0 || u >= W) continue;
                        const Vec3 d2 = (T.linear() * Vec3((u - out.frame.intrinsics.cx) / out.frame.intrinsics.fx, (y - out.frame.intrinsics.cy) / out.frame.intrinsics.fy, 1.0)).normalized();
                        const auto h2 = setup.scene.intersect(cam, d2);
                        if (!h2 || std::abs(h2->t - hit->t) > 10.0) sil = true;
                    }
                    sil_hist[sil ? 1 : 0]++;
                }
                if (!hit) no_hit++;
                const int band = near_edge(x, y) - 1;
                int k = 0;
                for (int dy = -2; dy <= 2; ++dy)
                    for (int dx = -2; dx <= 2; ++dx) {
                        const int u = x + dx, v = y + dy;
                        if (u < 0 || v < 0 || u >= W || v >= H) continue;
                        const float z = P(u, v).z();
                        if (z > 0 && std::abs(z - p.z()) > std::max(3.0f, 0.01f * p.z())) k = 2;
                        else if (z <= 0 && k == 0) k = 1;
                    }
                kind[k][0]++;
                kind[k][1] += err > 0.5;
                kind[k][2] += err > 2.0;
                counts[band][0]++;
                counts[band][1] += err > 0.5;
                counts[band][2] += err > 2.0;
            }
    }
    const auto pct = [](long n, long total) { return 100.0 * static_cast<double>(n) / static_cast<double>(std::max(1L, total)); };
    long tot = 0, g5 = 0, g2 = 0;
    for (int k = 0; k < 3; ++k) tot += kind[k][0], g5 += kind[k][1], g2 += kind[k][2];
    std::println("all: {} pixels, {} > 0.5 mm, {} > 2 mm", tot, g5, g2);
    std::println("disparity error: <0.25 px {}, 0.25-0.5 {}, 0.5-1 {}, >=1 px {} (by depth 150-300 {}, 300-450 {}, 450-600 {}, 600+ {})",
                 disp_hist[0], disp_hist[1], disp_hist[2], disp_hist[3], disp_z[0], disp_z[1], disp_z[2], disp_z[3]);
    for (int b = 0; b < 6; ++b)
        std::println("texture std {:2}-{:2}: good {:7}, bad (>=1 px) {:6} ({:.1f}% bad)", b * 5, b == 5 ? 255 : b * 5 + 5, tex[0][b], tex[1][b],
                     pct(tex[1][b], tex[0][b] + tex[1][b]));
    std::println("gross: {} too close, {} too far; {} within 6 px of a true silhouette, {} not; {} rays hit nothing", signed_hist[0], signed_hist[1],
                 sil_hist[1], sil_hist[0], no_hit);
    for (int k = 0; k < 3; ++k)
        std::println("{:12}: {:8} pixels, {:5.2f}% > 0.5 mm, {:5.2f}% > 2 mm", k == 0 ? "interior" : k == 1 ? "hole border" : "depth jump", kind[k][0],
                     pct(kind[k][1], kind[k][0]), pct(kind[k][2], kind[k][0]));
    for (int b = 0; b < 4; ++b)
        std::println("{} px from an edge: {:8} pixels, {:5.2f}% > 0.5 mm, {:5.2f}% > 2 mm", b < 3 ? std::to_string(b + 1) : std::string(">3"),
                     counts[b][0], pct(counts[b][1], counts[b][0]), pct(counts[b][2], counts[b][0]));
}
