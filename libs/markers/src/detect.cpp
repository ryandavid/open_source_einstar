#include "einstar/markers/detect.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Dense>
#include <tbb/parallel_for.h>

namespace einstar::markers {
namespace {

// Connected components (4-neighbourhood) of pixels above the threshold.
std::vector<Blob> find_blobs(ImageView<const std::uint8_t> img, int thr) {
    const int w = img.width, h = img.height;
    std::vector<std::int32_t> label(static_cast<std::size_t>(w * h), -1);
    std::vector<Blob> blobs;
    std::vector<int> stack;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const auto i = static_cast<std::size_t>(y * w + x);
            if (img(x, y) <= thr || label[i] >= 0) continue;
            Blob b{x, y, x, y, 0, 0};
            const int id = static_cast<int>(blobs.size());
            stack.push_back(y * w + x);
            label[i] = id;
            while (!stack.empty()) {
                const int p = stack.back();
                stack.pop_back();
                const int px = p % w, py = p / w;
                ++b.pixels;
                b.peak = std::max<int>(b.peak, img(px, py));
                b.x0 = std::min(b.x0, px), b.x1 = std::max(b.x1, px), b.y0 = std::min(b.y0, py), b.y1 = std::max(b.y1, py);
                const int nb[4][2] = {{px - 1, py}, {px + 1, py}, {px, py - 1}, {px, py + 1}};
                for (const auto& n : nb) {
                    if (n[0] < 0 || n[1] < 0 || n[0] >= w || n[1] >= h) continue;
                    const auto j = static_cast<std::size_t>(n[1] * w + n[0]);
                    if (label[j] >= 0 || img(n[0], n[1]) <= thr) continue;
                    label[j] = id;
                    stack.push_back(n[1] * w + n[0]);
                }
            }
            blobs.push_back(b);
        }
    return blobs;
}

// Marching-squares iso-contour points (edge crossings) of `level` inside a ROI.
std::vector<Vec2> iso_points(ImageView<const std::uint8_t> img, int x0, int y0, int x1, int y1, double level) {
    std::vector<Vec2> pts;
    for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x) {
            const double v = img(x, y), vr = img(x + 1, y), vd = img(x, y + 1);
            if ((v - level) * (vr - level) < 0) pts.emplace_back(x + (level - v) / (vr - v), y);
            if ((v - level) * (vd - level) < 0) pts.emplace_back(x, y + (level - v) / (vd - v));
        }
    return pts;
}

}  // namespace

std::vector<Blob> find_blobs_cpu(ImageView<const std::uint8_t> img, int threshold) { return find_blobs(img, threshold); }

bool fit_ellipse(const std::vector<Vec2>& pts, Ellipse& out) {
    if (pts.size() < 6) return false;
    // Normalise for conditioning.
    Vec2 mean = Vec2::Zero();
    for (const auto& p : pts) mean += p;
    mean /= static_cast<double>(pts.size());
    double scale = 0;
    for (const auto& p : pts) scale += (p - mean).norm();
    scale = scale / static_cast<double>(pts.size());
    if (scale < 1e-9) return false;
    // Fitzgibbon, Pilu & Fisher (1999), numerically stable variant (Halir & Flusser 1998).
    Eigen::MatrixXd D1(pts.size(), 3), D2(pts.size(), 3);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const double x = (pts[i].x() - mean.x()) / scale, y = (pts[i].y() - mean.y()) / scale;
        D1.row(static_cast<Eigen::Index>(i)) << x * x, x * y, y * y;
        D2.row(static_cast<Eigen::Index>(i)) << x, y, 1.0;
    }
    const Eigen::Matrix3d S1 = D1.transpose() * D1, S2 = D1.transpose() * D2, S3 = D2.transpose() * D2;
    Eigen::FullPivLU<Eigen::Matrix3d> lu(S3);
    if (!lu.isInvertible()) return false;
    const Eigen::Matrix3d T = -lu.inverse() * S2.transpose();
    Eigen::Matrix3d M = S1 + S2 * T;
    Eigen::Matrix3d Mc;
    Mc.row(0) = M.row(2) / 2.0;
    Mc.row(1) = -M.row(1);
    Mc.row(2) = M.row(0) / 2.0;
    Eigen::EigenSolver<Eigen::Matrix3d> es(Mc);
    int best = -1;
    Eigen::Vector3d a1;
    for (int k = 0; k < 3; ++k) {
        const Eigen::Vector3d v = es.eigenvectors().col(k).real();
        if (4 * v(0) * v(2) - v(1) * v(1) > 0) {
            best = k;
            a1 = v;
        }
    }
    if (best < 0) return false;
    const Eigen::Vector3d a2 = T * a1;
    // Conic A x^2 + B xy + C y^2 + D x + E y + F = 0 (normalised coordinates).
    const double A = a1(0), B = a1(1), C = a1(2), Dd = a2(0), E = a2(1), F = a2(2);
    const double den = B * B - 4 * A * C;
    if (den >= 0) return false;
    const double cx = (2 * C * Dd - B * E) / den, cy = (2 * A * E - B * Dd) / den;
    const double num = 2 * (A * E * E + C * Dd * Dd - B * Dd * E + den * F);
    const double root = std::sqrt((A - C) * (A - C) + B * B);
    const double s1 = -std::sqrt(std::max(0.0, num * (A + C + root))) / den;
    const double s2 = -std::sqrt(std::max(0.0, num * (A + C - root))) / den;
    if (!(s1 > 0) || !(s2 > 0)) return false;
    double theta = 0.5 * std::atan2(-B, C - A);
    double a = s1, b = s2;
    if (a < b) {
        std::swap(a, b);
        theta += M_PI / 2;
    }
    out.center = mean + scale * Vec2(cx, cy);
    out.a = a * scale;
    out.b = b * scale;
    out.angle = theta;
    // RMS geometric residual (radial approximation).
    double ss = 0;
    const double ct = std::cos(out.angle), st = std::sin(out.angle);
    for (const auto& p : pts) {
        const Vec2 d = p - out.center;
        const double u = ct * d.x() + st * d.y(), v = -st * d.x() + ct * d.y();
        const double r = std::sqrt(u * u + v * v);
        const double phi = std::atan2(v / out.b, u / out.a);
        const double re = std::hypot(out.a * std::cos(phi), out.b * std::sin(phi));
        ss += (r - re) * (r - re);
    }
    out.residual = std::sqrt(ss / static_cast<double>(pts.size()));
    out.points = static_cast<int>(pts.size());
    return true;
}

const char* reject_name(Reject r) {
    constexpr const char* names[] = {"accepted", "box size", "aspect", "fill", "border", "contrast", "contour",
                                     "coverage", "axis ratio", "residual", "ellipse size", "dark ring"};
    return names[static_cast<std::size_t>(r)];
}

std::vector<Ellipse> detect_markers(ImageView<const std::uint8_t> img, const DetectParams& p, FitStats* stats) {
    return fit_blobs(img, find_blobs(img, p.threshold), p, stats);
}

std::vector<Ellipse> fit_blobs(ImageView<const std::uint8_t> img, const std::vector<Blob>& blobs, const DetectParams& p, FitStats* stats) {
    std::vector<Ellipse> found(blobs.size());
    std::vector<std::uint8_t> ok(blobs.size(), 0);
    std::vector<Reject> why(blobs.size(), Reject::accepted);  // (per blob: fit_one may run in parallel)
    auto fit_one = [&](std::size_t bi) {
        const Blob& b = blobs[bi];
        auto reject = [&](Reject r) { why[bi] = r; };
        const int bw = b.x1 - b.x0 + 1, bh = b.y1 - b.y0 + 1;
        if (bw < p.min_diameter_px || bh < p.min_diameter_px || bw > p.max_diameter_px || bh > p.max_diameter_px) return reject(Reject::box_size);
        if (std::max(bw, bh) > p.max_aspect * std::min(bw, bh)) return reject(Reject::aspect);
        if (b.pixels < p.min_fill * bw * bh * M_PI / 4.0) return reject(Reject::fill);  // relative to the inscribed ellipse area
        const int m = std::max(3, std::max(bw, bh) / 2);
        const int x0 = b.x0 - m, y0 = b.y0 - m, x1 = b.x1 + m, y1 = b.y1 + m;
        if (x0 < p.border || y0 < p.border || x1 >= img.width - p.border || y1 >= img.height - p.border) return reject(Reject::border);
        // Local background: low percentile of the ROI ring.
        std::vector<int> ring;
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                if (x == x0 || x == x1 || y == y0 || y == y1) ring.push_back(img(x, y));
        std::nth_element(ring.begin(), ring.begin() + static_cast<std::ptrdiff_t>(ring.size() / 5), ring.end());
        const double bg = ring[ring.size() / 5];
        if (b.peak - bg < 40) return reject(Reject::contrast);
        const double level = 0.5 * (bg + b.peak);
        auto pts = iso_points(img, x0, y0, x1, y1, level);
        // Only the blob's own boundary: bright neighbours inside the ROI (speckle around a sticker,
        // other markers) would otherwise pull the fit.
        std::erase_if(pts, [&](const Vec2& q) { return q.x() < b.x0 - 2 || q.x() > b.x1 + 2 || q.y() < b.y0 - 2 || q.y() > b.y1 + 2; });
        Ellipse e;
        if (!fit_ellipse(pts, e)) return reject(Reject::contour);
        // Discard contour points of neighbouring structures, then refit.
        std::vector<Vec2> keep;
        const double ct = std::cos(e.angle), st = std::sin(e.angle);
        std::array<int, 12> sectors{};
        for (const auto& q : pts) {
            const Vec2 d = q - e.center;
            const double u = ct * d.x() + st * d.y(), v = -st * d.x() + ct * d.y();
            const double rr = std::sqrt((u / e.a) * (u / e.a) + (v / e.b) * (v / e.b));
            if (std::abs(rr - 1.0) * e.b < 1.0) {
                keep.push_back(q);
                const double ang = std::atan2(d.y(), d.x()) + M_PI;
                ++sectors[static_cast<std::size_t>(std::min(11, static_cast<int>(ang / (2 * M_PI) * 12)))];
            }
        }
        if (keep.size() < 8 || !fit_ellipse(keep, e)) return reject(Reject::contour);
        const int covered = static_cast<int>(std::ranges::count_if(sectors, [](int c) { return c > 0; }));
        if (covered < p.min_coverage * 12) return reject(Reject::coverage);
        if (e.a / e.b > p.max_axis_ratio) return reject(Reject::axis_ratio);
        if (e.residual > p.max_residual_px) return reject(Reject::residual);
        if (2 * e.b < p.min_diameter_px * 0.7 || 2 * e.a > p.max_diameter_px) return reject(Reject::ellipse_size);
        // Marker stickers have a dark ring around the reflective disc; laser speckle dots and specular
        // highlights sit among other bright structure.
        if (p.ring_scale > 0) {
            const bool saturated = b.peak >= p.saturated_level;
            const double ring_level = bg + (saturated ? p.saturated_ring_max_contrast : p.ring_max_contrast) * (b.peak - bg);
            const double max_bright = saturated ? p.saturated_max_ring_bright_fraction : p.max_ring_bright_fraction;
            int bright = 0, n = 0;
            const double ca = std::cos(e.angle), sa = std::sin(e.angle);
            for (int k = 0; k < 32; ++k) {
                const double t = 2 * M_PI * k / 32;
                const double u = p.ring_scale * e.a * std::cos(t), v = p.ring_scale * e.b * std::sin(t);
                const int x = static_cast<int>(std::lround(e.center.x() + ca * u - sa * v));
                const int y = static_cast<int>(std::lround(e.center.y() + sa * u + ca * v));
                if (x < 0 || y < 0 || x >= img.width || y >= img.height) continue;
                ++n;
                bright += img(x, y) > ring_level;
            }
            if (n < 16 || bright > max_bright * n) return reject(Reject::ring);
        }
        e.peak = b.peak;
        found[bi] = e;
        ok[bi] = 1;
    };
    // Few blobs (the GPU pre-selects them): a thread pool would cost more than the work.
    if (blobs.size() < 128)
        for (std::size_t bi = 0; bi < blobs.size(); ++bi) fit_one(bi);
    else
        tbb::parallel_for(std::size_t{0}, blobs.size(), fit_one);
    std::vector<Ellipse> out;
    for (std::size_t i = 0; i < found.size(); ++i)
        if (ok[i]) out.push_back(found[i]);
    if (stats)
        for (const Reject r : why) ++stats->count[static_cast<std::size_t>(r)];
    return out;
}

}  // namespace einstar::markers
