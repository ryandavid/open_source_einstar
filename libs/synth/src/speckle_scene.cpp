#include "einstar/synth/speckle_scene.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include <tbb/parallel_for.h>

namespace einstar::synth {
namespace {

std::optional<Hit> intersect_one(const Plane& p, const Vec3& o, const Vec3& d) {
    const double denom = p.normal.dot(d);
    if (std::abs(denom) < 1e-12) return std::nullopt;
    const double t = p.normal.dot(p.point - o) / denom;
    if (t <= 1e-6) return std::nullopt;
    const Vec3 n = denom < 0 ? p.normal : Vec3(-p.normal);
    return Hit{t, o + t * d, n.normalized()};
}

std::optional<Hit> intersect_one(const Sphere& s, const Vec3& o, const Vec3& d) {
    const Vec3 oc = o - s.center;
    const double b = oc.dot(d);
    const double c = oc.squaredNorm() - s.radius * s.radius;
    const double disc = b * b - c;
    if (disc < 0) return std::nullopt;
    const double sq = std::sqrt(disc);
    double t = -b - sq;
    if (t <= 1e-6) t = -b + sq;
    if (t <= 1e-6) return std::nullopt;
    const Vec3 p = o + t * d;
    return Hit{t, p, (p - s.center).normalized()};
}

std::optional<Hit> intersect_one(const Box& b, const Vec3& o, const Vec3& d) {
    const SE3 T_bw = b.T_world_box.inverse();
    const Vec3 lo = T_bw * o;
    const Vec3 ld = T_bw.linear() * d;
    double tmin = -1e30, tmax = 1e30;
    int axis = -1;
    double sign = 1;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(ld[i]) < 1e-12) {
            if (std::abs(lo[i]) > b.half_extent[i]) return std::nullopt;
            continue;
        }
        double t1 = (-b.half_extent[i] - lo[i]) / ld[i], t2 = (b.half_extent[i] - lo[i]) / ld[i];
        double s = -1;
        if (t1 > t2) {
            std::swap(t1, t2);
            s = 1;
        }
        if (t1 > tmin) {
            tmin = t1;
            axis = i;
            sign = s;
        }
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return std::nullopt;
    }
    if (tmin <= 1e-6 || axis < 0) return std::nullopt;
    Vec3 n_local = Vec3::Zero();
    n_local[axis] = sign;
    return Hit{tmin, o + tmin * d, (b.T_world_box.linear() * n_local).normalized()};
}

// Distorted pixel -> undistorted normalised ray (Newton on the Brown-Conrady model).
Vec2 pixel_to_normalized(const CameraModel& cam, double u, double v) {
    const bool has_dist = std::any_of(cam.dist.begin(), cam.dist.end(), [](double d) { return d != 0.0; });
    const double y0 = (v - cam.cy) / cam.fy;
    Vec2 n((u - cam.cx - cam.skew * y0) / cam.fx, y0);
    if (!has_dist) return n;
    const Vec2 target(u, v);
    constexpr double h = 1e-7;
    for (int i = 0; i < 10; ++i) {
        const Vec2 err = target - cam.distort_normalized(n);
        if (err.squaredNorm() < 1e-18) break;
        Eigen::Matrix2d J;
        J.col(0) = (cam.distort_normalized(n + Vec2(h, 0)) - cam.distort_normalized(n - Vec2(h, 0))) / (2 * h);
        J.col(1) = (cam.distort_normalized(n + Vec2(0, h)) - cam.distort_normalized(n - Vec2(0, h))) / (2 * h);
        n += J.inverse() * err;
    }
    return n;
}

}  // namespace

std::optional<Hit> Scene::intersect(const Vec3& origin, const Vec3& dir) const {
    std::optional<Hit> best;
    for (const auto& prim : primitives) {
        auto hit = std::visit([&](const auto& p) { return intersect_one(p, origin, dir); }, prim);
        if (hit && (!best || hit->t < best->t)) best = hit;
    }
    return best;
}

DotPattern DotPattern::random(int width, int height, int num_dots, double sigma_px, std::uint32_t seed) {
    DotPattern pat;
    pat.width = width;
    pat.height = height;
    pat.intensity = ImageF32(width, height, 0.0f);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(0.0, width), uy(0.0, height);
    const int r = static_cast<int>(std::ceil(3 * sigma_px));
    const double inv2s2 = 1.0 / (2 * sigma_px * sigma_px);
    for (int i = 0; i < num_dots; ++i) {
        const double cx = ux(rng), cy = uy(rng);
        const int x0 = static_cast<int>(cx), y0 = static_cast<int>(cy);
        for (int y = std::max(0, y0 - r); y <= std::min(height - 1, y0 + r); ++y)
            for (int x = std::max(0, x0 - r); x <= std::min(width - 1, x0 + r); ++x) {
                const double dx = x + 0.5 - cx, dy = y + 0.5 - cy;
                pat.intensity(x, y) += static_cast<float>(std::exp(-(dx * dx + dy * dy) * inv2s2));
            }
    }
    for (auto& v : pat.intensity.pixels()) v = std::min(v, 1.0f);
    return pat;
}

float DotPattern::sample(double u, double v) const {
    const double x = u - 0.5, y = v - 0.5;
    const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= width || y0 + 1 >= height) return 0.0f;
    const float fx = static_cast<float>(x - x0), fy = static_cast<float>(y - y0);
    const float a = intensity(x0, y0), b = intensity(x0 + 1, y0);
    const float c = intensity(x0, y0 + 1), d = intensity(x0 + 1, y0 + 1);
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
}

RenderedView render_view(const Scene& scene, const Projector& proj, const CameraModel& cam, const SE3& T_world_camera,
                         const RenderParams& params) {
    RenderedView out{ImageU8(cam.width, cam.height, 0), ImageF32(cam.width, cam.height, 0.0f)};
    const SE3 T_projector_world = proj.T_world_projector.inverse();
    const Vec3 proj_center = proj.T_world_projector.translation();
    const Vec3 cam_center = T_world_camera.translation();
    const Mat3 R = T_world_camera.linear();
    const int ss = std::max(1, params.supersample);

    tbb::parallel_for(0, cam.height, [&](int y) {
        std::mt19937 rng(params.seed * 7919u + static_cast<std::uint32_t>(y));
        std::normal_distribution<double> noise(0.0, params.noise_sigma);
        for (int x = 0; x < cam.width; ++x) {
            double acc = 0.0;
            int hits = 0;
            for (int sy = 0; sy < ss; ++sy)
                for (int sx = 0; sx < ss; ++sx) {
                    const double u = x + (sx + 0.5) / ss, v = y + (sy + 0.5) / ss;
                    const Vec2 nrm = pixel_to_normalized(cam, u - 0.5, v - 0.5);  // pixel centres at integers
                    const Vec3 dir_cam(nrm.x(), nrm.y(), 1.0);
                    const Vec3 dir = (R * dir_cam).normalized();
                    const auto hit = scene.intersect(cam_center, dir);
                    if (!hit) continue;
                    ++hits;
                    if (sx == ss / 2 && sy == ss / 2) {
                        out.depth(x, y) = static_cast<float>((T_world_camera.inverse() * hit->point).z());
                    }
                    // Projector illumination with shadowing.
                    const Vec3 to_proj = proj_center - hit->point;
                    const double dist = to_proj.norm();
                    const Vec3 l = to_proj / dist;
                    double lit = 0.0;
                    const auto blocker = scene.intersect(hit->point + 1e-4 * l, l);
                    if (!blocker || blocker->t > dist - 1e-3) {
                        const Vec3 pp = T_projector_world * hit->point;
                        if (pp.z() > 0) {
                            const double pu = proj.model.fx * pp.x() / pp.z() + proj.model.cx;
                            const double pv = proj.model.fy * pp.y() / pp.z() + proj.model.cy;
                            lit = proj.power * proj.pattern.sample(pu, pv) * std::max(0.0, hit->normal.dot(l));
                        }
                    }
                    acc += params.ambient + params.albedo * lit;
                }
            if (hits == 0) continue;
            const double value = 255.0 * acc / (ss * ss) + noise(rng);
            out.image(x, y) = static_cast<std::uint8_t>(std::clamp(std::lround(value), 0L, 255L));
        }
    });
    return out;
}

}  // namespace einstar::synth
