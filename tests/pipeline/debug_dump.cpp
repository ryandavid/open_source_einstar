// Debug helper (not a test): renders one emulated frame pair and writes raw/rectified images + depth as PGM.
#include <algorithm>
#include <cstdio>
#include <format>
#include <vector>
#include <fstream>
#include <print>
#include <string>

#include "einstar/calib/device_calibration.hpp"
#include "einstar/pipeline/stereo_frontend.hpp"
#include "einstar/synth/speckle_scene.hpp"
#include "einstar/depth/stereo.hpp"

using namespace einstar;

static void pgm(const std::string& path, const ImageU8& img) {
    std::ofstream f(path, std::ios::binary);
    f << "P5\n" << img.width() << " " << img.height() << "\n255\n";
    f.write(reinterpret_cast<const char*>(img.data()), static_cast<std::streamsize>(img.size()));
}

int main(int argc, char** argv) {
    const std::string out = argc > 1 ? argv[1] : ".";
    auto cal = calib::load_ccf_directory(EINSTAR_TEST_CALIBRATION_DIR);
    if (!cal) return 1;
    const auto rig = cal->rig();
    const SE3 T_left_right = rig.T_right_left.inverse();
    synth::Scene scene;
    scene.primitives.push_back(synth::Plane{Vec3(0, 70, 0), Vec3(0, -1, 0)});
    scene.primitives.push_back(synth::Sphere{Vec3(-80, 10, 20), 55.0});
    synth::Projector proj;
    proj.model.fx = proj.model.fy = 800;
    proj.model.cx = 640;
    proj.model.cy = 400;
    proj.pattern = synth::DotPattern::random(1280, 800, 9000, 3.5, 11);
    // Left camera 300 mm in front of the sphere, toed in by half the rig angle.
    SE3 T_wl = SE3::Identity();
    T_wl.linear() = Eigen::AngleAxisd(11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    T_wl.translation() = Vec3(-80 - 80, 0, -280);
    const SE3 T_wr = T_wl * T_left_right;
    proj.T_world_projector = T_wl;
    proj.T_world_projector.translation() = T_wl * (0.5 * T_left_right.translation());
    proj.T_world_projector.linear() = T_wl.linear() * Eigen::AngleAxisd(-11.07 * M_PI / 180, Vec3::UnitY()).toRotationMatrix();
    synth::RenderParams rp;
    rp.supersample = 1;
    const auto l = synth::render_view(scene, proj, rig.left, T_wl, rp);
    const auto r = synth::render_view(scene, proj, rig.right, T_wr, rp);
    pgm(out + "/raw_left.pgm", l.image);
    pgm(out + "/raw_right.pgm", r.image);
    pipeline::StereoFrontendParams fp_cpu_previews;
    fp_cpu_previews.cpu_previews = true;
    pipeline::StereoFrontend fe(rig, fp_cpu_previews);
    const auto d = fe.process(l.image, r.image);
    pgm(out + "/rect_left.pgm", d.rectified_left);
    pgm(out + "/rect_right.pgm", d.rectified_right);
    ImageU8 depth(d.frame.points.width(), d.frame.points.height(), 0);
    int valid = 0;
    for (int y = 0; y < depth.height(); ++y)
        for (int x = 0; x < depth.width(); ++x) {
            const float z = d.frame.points(x, y).z();
            if (z > 0) {
                ++valid;
                depth(x, y) = static_cast<std::uint8_t>(std::clamp((z - 150.0f) / 2.5f, 1.0f, 255.0f));
            }
        }
    pgm(out + "/depth.pgm", depth);
    // Stage-by-stage coverage.
    const auto q_l = depth::downsample2(d.rectified_left.view());
    const auto q_r = depth::downsample2(d.rectified_right.view());
    const auto& g = fe.rectification().geometry;
    // Depth accuracy of the full frontend at half resolution against the rendered depth.
    {
        const auto& R = fe.rectification();
        const auto& K = fe.depth_intrinsics();
        const auto map = calib::build_remap(rig.left, R.R_left, R.rectified);
        std::vector<double> errs;
        double bin_sum[4][8] = {}, bin_rel[4][8] = {};
        int bin_n[4][8] = {};
        for (int y = 0; y < K.height; ++y)
            for (int x = 0; x < K.width; ++x) {
                const Vec3f& p = d.frame.points(x, y);
                if (p.z() <= 0) continue;
                // The rectified-left ray through this half-res pixel, and the true depth along it.
                const double fx = 2.0 * x + 0.5, fy = 2.0 * y + 0.5;
                const float sx = map.map_x(static_cast<int>(fx), static_cast<int>(fy));
                const float sy = map.map_y(static_cast<int>(fx), static_cast<int>(fy));
                const int ix = static_cast<int>(std::lround(sx)), iy = static_cast<int>(std::lround(sy));
                if (ix < 0 || iy < 0 || ix >= 1280 || iy >= 1024 || l.depth(ix, iy) <= 0) continue;
                const Vec3 ray_rect((fx - R.rectified.cx) / R.rectified.fx, (fy - R.rectified.cy) / R.rectified.fy, 1.0);
                const Vec3 ray_left = R.R_left.transpose() * ray_rect;
                const double z_rect = (R.R_left * (ray_left * (l.depth(ix, iy) / ray_left.z()))).z();
                errs.push_back(p.z() - z_rect);
                const int bx = x * 8 / K.width, by = y * 4 / K.height;
                bin_sum[by][bx] += p.z() - z_rect;
                bin_rel[by][bx] += (p.z() - z_rect) / z_rect * 1000.0;
                ++bin_n[by][bx];
            }
        std::ranges::sort(errs);
        std::println("mean depth error (mm) and relative (per mille) by image region (8x4 bins, - = no data):");
        for (int by = 0; by < 4; ++by) {
            std::string line;
            for (int bx = 0; bx < 8; ++bx)
                line += bin_n[by][bx] > 50 ? std::format("{:+6.3f}/{:+5.2f} ", bin_sum[by][bx] / bin_n[by][bx], bin_rel[by][bx] / bin_n[by][bx])
                                           : std::string("      -      ");
            std::println("  {}", line);
        }
        if (!errs.empty())
            std::println("frontend depth vs truth: {} px, median signed err {:.3f} mm, p10 {:.3f} p90 {:.3f}", errs.size(),
                         errs[errs.size() / 2], errs[errs.size() / 10], errs[errs.size() * 9 / 10]);
    }
    // Ground-truth disparity at quarter res from the rendered left depth.
    {
        const auto& R = fe.rectification();
        const auto map = calib::build_remap(rig.left, R.R_left, R.rectified);
        ImageF32 truth(320, 256, -1.0f);
        for (int y = 0; y < 256; ++y)
            for (int x = 0; x < 320; ++x) {
                const int fx = 4 * x + 2, fy = 4 * y + 2;  // quarter px centre in full-res rectified
                const float sx = map.map_x(fx, fy), sy = map.map_y(fx, fy);
                const int ix = static_cast<int>(std::lround(sx)), iy = static_cast<int>(std::lround(sy));
                if (ix < 0 || iy < 0 || ix >= 1280 || iy >= 1024) continue;
                const float z = l.depth(ix, iy);
                if (z <= 0) continue;
                const Vec3 ray_rect((fx - R.rectified.cx) / R.rectified.fx, (fy - R.rectified.cy) / R.rectified.fy, 1.0);
                const Vec3 ray_left = R.R_left.transpose() * ray_rect;
                const Vec3 p_left = ray_left * (z / ray_left.z());
                const double z_rect = (R.R_left * p_left).z();
                // Visible from the right camera too?
                const Vec3 p_right = rig.T_right_left * p_left;
                const Vec2 pr = rig.right.project(p_right);
                const int rx = static_cast<int>(std::lround(pr.x())), ry = static_cast<int>(std::lround(pr.y()));
                if (rx < 0 || ry < 0 || rx >= 1280 || ry >= 1024 || std::abs(r.depth(rx, ry) - p_right.z()) > 2.0) continue;
                truth(x, y) = static_cast<float>(R.geometry.disparity_from_depth(z_rect) / 4.0);
            }
        depth::SgmParams sp;
        sp.min_disparity = static_cast<int>(std::floor(g.disparity_from_depth(700) / 4)) - 2;
        sp.num_disparities = static_cast<int>(g.disparity_from_depth(150) / 4) - sp.min_disparity + 4;
        struct V { const char* name; int p1, p2, cr; bool eight, slant; float uniq; int lr; };
        for (const V var : {V{"base", 8, 96, 2, false, false, 0.9f, 1}, V{"slant", 8, 96, 2, false, true, 0.9f, 1},
                            V{"slant p1=2", 2, 96, 2, false, true, 0.9f, 1}, V{"slant 8path", 8, 96, 2, true, true, 0.9f, 1},
                            V{"slant 3x3", 4, 64, 1, false, true, 0.9f, 1}, V{"slant uniq.97", 8, 96, 2, false, true, 0.97f, 1},
                            V{"slant lr2", 8, 96, 2, false, true, 0.9f, 2}, V{"slant 8p lr2 u.95", 8, 96, 2, true, true, 0.95f, 2}}) {
        sp.p1 = var.p1; sp.p2 = var.p2; sp.census_radius_x = sp.census_radius_y = var.cr; sp.eight_paths = var.eight;
        sp.slant_steps = var.slant; sp.uniqueness = var.uniq; sp.lr_max_diff = var.lr;
        const auto sgm = depth::sgm_disparity(q_l.view(), q_r.view(), sp, true);
        int t = 0, v = 0, good = 0, matchable = 0;
        double bias = 0;
        for (int y = 0; y < 256; ++y)
            for (int x = 0; x < 320; ++x) {
                if (truth(x, y) < 0) continue;
                ++t;
                if (static_cast<float>(x) - truth(x, y) < 3.0f) continue;
                ++matchable;
                if (sgm(x, y) < 0) continue;
                ++v;
                if (std::abs(sgm(x, y) - truth(x, y)) < 1.0f) { ++good; bias += sgm(x, y) - truth(x, y); }
            }
        if (std::string(var.name) == "base") {
            std::ofstream f(out + "/sgm_eval.ppm", std::ios::binary);
            f << "P6\n320 256\n255\n";
            for (int y = 0; y < 256; ++y)
                for (int x = 0; x < 320; ++x) {
                    std::uint8_t c[3] = {0, 0, 0};
                    const std::uint8_t grey = q_l(x, y) / 2;
                    c[0] = c[1] = c[2] = grey;
                    if (truth(x, y) >= 0 && static_cast<float>(x) - truth(x, y) >= 3.0f) {
                        if (sgm(x, y) < 0) { c[0] = 40; c[1] = 60; c[2] = 220; }
                        else if (std::abs(sgm(x, y) - truth(x, y)) < 1.0f) { c[0] = 40; c[1] = 200; c[2] = 60; }
                        else { c[0] = 230; c[1] = 40; c[2] = 40; }
                    }
                    f.write(reinterpret_cast<const char*>(c), 3);
                }
        }
        std::println("{:18} matchable {}, SGM valid {} ({:.0f}%), within 1 px {} ({:.1f}% of valid)", var.name, matchable, v,
                     100.0 * v / matchable, good, 100.0 * good / std::max(1, v));
        (void)t; (void)bias;
        }
    }
    for (const auto [min_zncc, radius, speckle] : {std::tuple{0.5f, 2, 100}, std::tuple{0.5f, 2, 0}, std::tuple{0.2f, 2, 0},
                                                   std::tuple{0.5f, 4, 0}}) {
        depth::StereoParams p;
        p.pyramid_levels = 1;
        p.sgm.min_disparity = static_cast<int>(std::floor(g.disparity_from_depth(700) / 4)) - 2;
        p.sgm.num_disparities = static_cast<int>(g.disparity_from_depth(150) / 4) - p.sgm.min_disparity + 4;
        p.refine.min_zncc = min_zncc;
        p.refine.search_radius = radius;
        p.speckle.max_region_size = speckle;
        const auto res = depth::compute_disparity(d.rectified_left.view(), d.rectified_right.view(), p);
        int n = 0;
        for (float v : res.disparity.pixels()) n += v >= 0;
        std::println("refine zncc>={:.1f} radius {} speckle {:3}: {} valid of {}", min_zncc, radius, speckle, n, res.disparity.size());
    }
    for (float uniq : {0.9f, 0.97f, 1.01f}) {
        for (int lr : {1, 3, 1000}) {
            depth::SgmParams sp;
            sp.min_disparity = static_cast<int>(std::floor(g.disparity_from_depth(700) / 4)) - 2;
            sp.num_disparities = static_cast<int>(g.disparity_from_depth(150) / 4) - sp.min_disparity + 4;
            sp.uniqueness = uniq;
            sp.lr_max_diff = lr;
            const auto sgm = depth::sgm_disparity(q_l.view(), q_r.view(), sp, true);
            int n = 0;
            for (float v : sgm.pixels()) n += v >= 0;
            std::println("SGM quarter-res uniq {:.2f} lr {:4}: {} valid of {}", uniq, lr, n, sgm.size());
            if (uniq > 1.0f && lr == 1000) {
                ImageU8 vis(sgm.width(), sgm.height(), 0);
                for (int y = 0; y < vis.height(); ++y)
                    for (int x = 0; x < vis.width(); ++x)
                        if (depth::valid_disparity(sgm(x, y))) vis(x, y) = static_cast<std::uint8_t>(std::clamp((sgm(x, y) - static_cast<float>(sp.min_disparity)) * 255.0f / static_cast<float>(sp.num_disparities), 1.0f, 255.0f));
                pgm(out + "/sgm_raw.pgm", vis);
                pgm(out + "/q_left.pgm", q_l);
                pgm(out + "/q_right.pgm", q_r);
            }
        }
    }
    std::println("valid depth {} px, stereo {:.1f} ms", valid, d.stereo_ms);
}
