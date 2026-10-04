#include "einstar/depth/stereo.hpp"

#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include <tbb/parallel_for.h>

namespace einstar::depth {
namespace {

using Cost = std::uint16_t;
constexpr Cost kMaxCost = std::numeric_limits<Cost>::max();

template <typename F>
void parallel_rows(int n, F&& f) {
    tbb::parallel_for(tbb::blocked_range<int>(0, n), [&](const tbb::blocked_range<int>& r) {
        for (int i = r.begin(); i != r.end(); ++i) f(i);
    });
}

// Dense per-pixel cost storage: [y][x][d].
struct CostVolume {
    int w = 0, h = 0, d = 0;
    std::vector<Cost> data;

    CostVolume(int w_, int h_, int d_, Cost fill)
        : w(w_), h(h_), d(d_), data(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_) * static_cast<std::size_t>(d_), fill) {}
    [[nodiscard]] Cost* at(int x, int y) { return data.data() + offset(x, y); }
    [[nodiscard]] const Cost* at(int x, int y) const { return data.data() + offset(x, y); }

private:
    [[nodiscard]] std::size_t offset(int x, int y) const {
        assert(x >= 0 && x < w && y >= 0 && y < h);
        return (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * static_cast<std::size_t>(d);
    }
};

// Aggregates matching costs along one scanline direction and adds into `sum`.
// The path walks from (x0, y0) stepping (dx, dy) until leaving the image.
void aggregate_path(const CostVolume& cost, ImageView<const std::uint8_t> guide, CostVolume& sum, int x0, int y0,
                    int dx, int dy, const SgmParams& p, std::vector<Cost>& prev, std::vector<Cost>& cur) {
    const auto nd = static_cast<std::size_t>(cost.d);
    int x = x0, y = y0;
    const Cost* c = cost.at(x, y);
    Cost prev_min = kMaxCost;
    for (std::size_t k = 0; k < nd; ++k) {
        prev[k] = c[k];
        prev_min = std::min(prev_min, c[k]);
    }
    {
        Cost* s = sum.at(x, y);
        for (std::size_t k = 0; k < nd; ++k) s[k] = static_cast<Cost>(s[k] + prev[k]);
    }
    int prev_intensity = guide(x, y);
    x += dx;
    y += dy;
    while (x >= 0 && y >= 0 && x < cost.w && y < cost.h) {
        const int intensity = guide(x, y);
        int p2 = p.p2;
        if (p.adaptive_p2) p2 = std::max(p.p1 + 1, p.p2 / (std::abs(intensity - prev_intensity) / 8 + 1));
        prev_intensity = intensity;

        c = cost.at(x, y);
        const int jump = prev_min + p2;
        Cost cur_min = kMaxCost;
        for (std::size_t k = 0; k < nd; ++k) {
            int best = std::min<int>(prev[k], jump);
            if (k > 0) best = std::min<int>(best, prev[k - 1] + p.p1);
            if (k + 1 < nd) best = std::min<int>(best, prev[k + 1] + p.p1);
            if (p.slant_steps) {
                if (k > 1) best = std::min<int>(best, prev[k - 2] + p.p1);
                if (k + 2 < nd) best = std::min<int>(best, prev[k + 2] + p.p1);
            }
            const Cost v = static_cast<Cost>(c[k] + best - prev_min);
            cur[k] = v;
            cur_min = std::min(cur_min, v);
        }
        Cost* s = sum.at(x, y);
        for (std::size_t k = 0; k < nd; ++k) s[k] = static_cast<Cost>(s[k] + cur[k]);
        std::swap(prev, cur);
        prev_min = cur_min;
        x += dx;
        y += dy;
    }
}

// Parabola fit through three costs; returns offset in [-0.5, 0.5].
float subpixel_offset(float cm, float c0, float cp) {
    const float denom = cm - 2.0f * c0 + cp;
    if (denom <= 0.0f) return 0.0f;
    return std::clamp(0.5f * (cm - cp) / denom, -0.5f, 0.5f);
}

// ZNCC between a square window in `a` and a window in `b` warped by a disparity plane
// d(u, v) = d + gx*u + gy*v (bilinear sampling in `b`). Slanted windows are essential for the
// Einstar's wide, toed-in baseline, where inclined surfaces are strongly foreshortened.
float zncc_slanted(ImageView<const std::uint8_t> a, ImageView<const std::uint8_t> b, int x, int y, float d, float gx, float gy,
                   int r) {
    double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    const int n = (2 * r + 1) * (2 * r + 1);
    for (int v = -r; v <= r; ++v) {
        const std::uint8_t* ra = a.row(y + v);
        const std::uint8_t* rb = b.row(y + v);
        for (int u = -r; u <= r; ++u) {
            const float xb = static_cast<float>(x + u) - (d + gx * static_cast<float>(u) + gy * static_cast<float>(v));
            const int x0 = static_cast<int>(std::floor(xb));
            if (x0 < 0 || x0 + 1 >= b.width) return -2.0f;
            const float f = xb - static_cast<float>(x0);
            const double ib = rb[x0] * (1.0f - f) + rb[x0 + 1] * f;
            const double ia = ra[x + u];
            sa += ia;
            sb += ib;
            saa += ia * ia;
            sbb += ib * ib;
            sab += ia * ib;
        }
    }
    const double va = saa - sa * sa / n;
    const double vb = sbb - sb * sb / n;
    if (va < 1e-6 || vb < 1e-6) return -1.0f;
    return static_cast<float>((sab - sa * sb / n) / std::sqrt(va * vb));
}

// 3x3 median of valid neighbours (removes isolated SGM outliers before slopes are estimated).
ImageF32 median3(const ImageF32& in) {
    ImageF32 out(in.width(), in.height(), kInvalidDisparity);
    parallel_rows(in.height(), [&](int y) {
        float buf[9];
        for (int x = 0; x < in.width(); ++x) {
            if (!valid_disparity(in(x, y))) continue;
            int n = 0;
            for (int v = -1; v <= 1; ++v)
                for (int u = -1; u <= 1; ++u) {
                    const int xx = x + u, yy = y + v;
                    if (xx < 0 || yy < 0 || xx >= in.width() || yy >= in.height() || !valid_disparity(in(xx, yy))) continue;
                    buf[n++] = in(xx, yy);
                }
            std::nth_element(buf, buf + n / 2, buf + n);
            out(x, y) = buf[n / 2];
        }
    });
    return out;
}

// Re-estimates disparity at the next finer level around 2x the coarse estimate, with windows
// slanted by the coarse disparity gradient.
void refine_level(ImageView<const std::uint8_t> left, ImageView<const std::uint8_t> right, const ImageF32& coarse_raw,
                  const RefineParams& p, bool subpixel, ImageF32& out_disp, ImageF32& out_conf) {
    const int w = left.width, h = left.height;
    out_disp = ImageF32(w, h, kInvalidDisparity);
    out_conf = ImageF32(w, h, 0.0f);
    const ImageF32 coarse = median3(coarse_raw);
    const int cw = coarse.width(), ch = coarse.height();
    // Disparity gradient per fine pixel = coarse gradient (both disparity and x scale by 2).
    auto slope = [&](int cx, int cy, float& gx, float& gy) {
        gx = gy = 0.0f;
        const float c = coarse(cx, cy);
        if (cx > 0 && cx + 1 < cw && valid_disparity(coarse(cx - 1, cy)) && valid_disparity(coarse(cx + 1, cy))) {
            const float g = 0.5f * (coarse(cx + 1, cy) - coarse(cx - 1, cy));
            if (std::abs(coarse(cx + 1, cy) - c) < 2 && std::abs(c - coarse(cx - 1, cy)) < 2) gx = g;
        }
        if (cy > 0 && cy + 1 < ch && valid_disparity(coarse(cx, cy - 1)) && valid_disparity(coarse(cx, cy + 1))) {
            const float g = 0.5f * (coarse(cx, cy + 1) - coarse(cx, cy - 1));
            if (std::abs(coarse(cx, cy + 1) - c) < 2 && std::abs(c - coarse(cx, cy - 1)) < 2) gy = g;
        }
        gx = std::clamp(gx, -0.9f, 0.9f);
        gy = std::clamp(gy, -0.9f, 0.9f);
    };
    const int r = p.zncc_radius;
    constexpr float kStep = 0.5f;
    parallel_rows(h, [&](int y) {
        if (y < r || y >= h - r) return;
        const int cy = std::min(y / 2, ch - 1);
        float scores[128];
        for (int x = r; x < w - r; ++x) {
            const int cx = std::min(x / 2, cw - 1);
            const float d0 = coarse(cx, cy);
            if (!valid_disparity(d0)) continue;
            float gx, gy;
            slope(cx, cy, gx, gy);
            const float center = 2.0f * d0;
            const int nd = std::min(127, static_cast<int>(static_cast<float>(2 * p.search_radius) / kStep) + 1);
            const float lo = center - static_cast<float>(p.search_radius);
            float best = -2.0f;
            int best_k = -1;
            for (int k = 0; k < nd; ++k) {
                const float d = lo + kStep * static_cast<float>(k);
                scores[k] = zncc_slanted(left, right, x, y, d, gx, gy, r);
                if (scores[k] > best) {
                    best = scores[k];
                    best_k = k;
                }
            }
            if (best_k < 0 || best < p.min_zncc) continue;
            float d = lo + kStep * static_cast<float>(best_k);
            if (subpixel && best_k > 0 && best_k + 1 < nd && scores[best_k - 1] > -2.0f && scores[best_k + 1] > -2.0f)
                d += kStep * subpixel_offset(1.0f - scores[best_k - 1], 1.0f - best, 1.0f - scores[best_k + 1]);
            out_disp(x, y) = d;
            out_conf(x, y) = best;
        }
    });
}

}  // namespace

Image<std::uint64_t> census_transform(ImageView<const std::uint8_t> img, int rx, int ry) {
    Image<std::uint64_t> out(img.width, img.height, 0);
    parallel_rows(img.height, [&](int y) {
        if (y < ry || y >= img.height - ry) return;
        for (int x = rx; x < img.width - rx; ++x) {
            const std::uint8_t center = img(x, y);
            std::uint64_t bits = 0;
            for (int v = -ry; v <= ry; ++v) {
                const std::uint8_t* row = img.row(y + v);
                for (int u = -rx; u <= rx; ++u) {
                    if (u == 0 && v == 0) continue;
                    bits = (bits << 1) | static_cast<std::uint64_t>(row[x + u] < center);
                }
            }
            out(x, y) = bits;
        }
    });
    return out;
}

ImageU8 downsample2(ImageView<const std::uint8_t> img) {
    const int w = img.width / 2, h = img.height / 2;
    ImageU8 out(w, h);
    parallel_rows(h, [&](int y) {
        const std::uint8_t* r0 = img.row(2 * y);
        const std::uint8_t* r1 = img.row(2 * y + 1);
        for (int x = 0; x < w; ++x) {
            out(x, y) = static_cast<std::uint8_t>((r0[2 * x] + r0[2 * x + 1] + r1[2 * x] + r1[2 * x + 1] + 2) / 4);
        }
    });
    return out;
}

ImageF32 sgm_disparity(ImageView<const std::uint8_t> left, ImageView<const std::uint8_t> right, const SgmParams& p,
                       bool subpixel) {
    const int w = left.width, h = left.height, nd = p.num_disparities;
    const auto cl = census_transform(left, p.census_radius_x, p.census_radius_y);
    const auto cr = census_transform(right, p.census_radius_x, p.census_radius_y);
    const Cost invalid_cost = static_cast<Cost>((2 * p.census_radius_x + 1) * (2 * p.census_radius_y + 1));

    CostVolume cost(w, h, nd, invalid_cost);
    parallel_rows(h, [&](int y) {
        for (int x = 0; x < w; ++x) {
            Cost* c = cost.at(x, y);
            const std::uint64_t a = cl(x, y);
            for (int k = 0; k < nd; ++k) {
                const int xr = x - (p.min_disparity + k);
                if (xr < 0) break;
                if (xr >= w) continue;
                c[k] = static_cast<Cost>(std::popcount(a ^ cr(xr, y)));
            }
        }
    });

    // Aggregate. Each path direction is processed independently per scanline; paths sharing a
    // direction touch disjoint pixels so they can run in parallel, but different directions
    // write the same sums, so directions are sequential.
    CostVolume sum(w, h, nd, 0);
    struct Dir { int dx, dy; };
    std::vector<Dir> dirs = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    if (p.eight_paths) dirs.insert(dirs.end(), {{1, 1}, {-1, -1}, {1, -1}, {-1, 1}});
    for (const Dir dir : dirs) {
        // Starting pixels: every pixel on the entry border(s) for this direction.
        std::vector<std::pair<int, int>> starts;
        if (dir.dx != 0) {
            const int sx = dir.dx > 0 ? 0 : w - 1;
            for (int y = 0; y < h; ++y) starts.emplace_back(sx, y);
        }
        if (dir.dy != 0) {
            const int sy = dir.dy > 0 ? 0 : h - 1;
            for (int x = 0; x < w; ++x) {
                if (dir.dx != 0 && x == (dir.dx > 0 ? 0 : w - 1)) continue;  // corner already added
                starts.emplace_back(x, sy);
            }
        }
        tbb::parallel_for(tbb::blocked_range<std::size_t>(0, starts.size()), [&](const tbb::blocked_range<std::size_t>& r) {
            std::vector<Cost> prev(static_cast<std::size_t>(nd)), cur(static_cast<std::size_t>(nd));
            for (std::size_t i = r.begin(); i != r.end(); ++i) {
                aggregate_path(cost, left, sum, starts[i].first, starts[i].second, dir.dx, dir.dy, p, prev, cur);
            }
        });
    }

    // Winner-takes-all (left) plus the right-referenced disparity from the same volume.
    ImageF32 disp(w, h, kInvalidDisparity);
    constexpr int kNoMatch = std::numeric_limits<int>::min();
    Image<int> right_disp(w, h, kNoMatch);
    parallel_rows(h, [&](int y) {
        for (int x = 0; x < w; ++x) {
            const Cost* s = sum.at(x, y);
            // Candidates whose right pixel x - (min_disparity + k) lies inside the image.
            const int min_k = std::max(0, x - p.min_disparity - (w - 1));
            const int max_k = std::min(nd, x - p.min_disparity + 1);
            if (max_k <= min_k) continue;
            int best_k = min_k;
            for (int k = min_k + 1; k < max_k; ++k)
                if (s[k] < s[best_k]) best_k = k;
            Cost second = kMaxCost;
            for (int k = min_k; k < max_k; ++k)
                if (std::abs(k - best_k) > 1) second = std::min(second, s[k]);
            if (second != kMaxCost && static_cast<float>(s[best_k]) >= p.uniqueness * static_cast<float>(second)) continue;
            float d = static_cast<float>(p.min_disparity + best_k);
            if (subpixel && best_k > min_k && best_k + 1 < max_k)
                d += subpixel_offset(s[best_k - 1], s[best_k], s[best_k + 1]);
            disp(x, y) = d;
        }
        for (int xr = 0; xr < w; ++xr) {
            int best_k = -1;
            Cost best = kMaxCost;
            for (int k = std::max(0, -(xr + p.min_disparity)); k < nd; ++k) {
                const int xl = xr + p.min_disparity + k;
                if (xl >= w) break;
                const Cost v = sum.at(xl, y)[k];
                if (v < best) {
                    best = v;
                    best_k = k;
                }
            }
            right_disp(xr, y) = best_k < 0 ? kNoMatch : p.min_disparity + best_k;
        }
    });

    // Left/right consistency.
    parallel_rows(h, [&](int y) {
        for (int x = 0; x < w; ++x) {
            const float d = disp(x, y);
            if (!valid_disparity(d)) continue;
            const int xr = static_cast<int>(std::lround(static_cast<float>(x) - d));
            if (xr < 0 || xr >= w || right_disp(xr, y) == kNoMatch ||
                std::abs(static_cast<float>(right_disp(xr, y)) - d) > static_cast<float>(p.lr_max_diff)) {
                disp(x, y) = kInvalidDisparity;
            }
        }
    });
    return disp;
}

void remove_speckles(ImageF32& disparity, const SpeckleParams& p) {
    const int w = disparity.width(), h = disparity.height();
    ImageU8 seen(w, h, 0);
    std::vector<int> stack;
    std::vector<int> region;  // pixels as y * w + x
    for (int start = 0; start < w * h; ++start) {
        if (seen.data()[start] || !valid_disparity(disparity.data()[start])) continue;
        region.clear();
        stack.push_back(start);
        seen.data()[start] = 1;
        while (!stack.empty()) {
            const int i = stack.back();
            stack.pop_back();
            region.push_back(i);
            const int x = i % w, y = i / w;
            const float d = disparity.data()[i];
            const int nbrs[4][2] = {{x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}};
            for (const auto& n : nbrs) {
                if (n[0] < 0 || n[1] < 0 || n[0] >= w || n[1] >= h) continue;
                const int j = n[1] * w + n[0];
                const float dj = disparity.data()[j];
                if (seen.data()[j] || !valid_disparity(dj) || std::abs(dj - d) > p.max_diff) continue;
                seen.data()[j] = 1;
                stack.push_back(j);
            }
        }
        if (static_cast<int>(region.size()) < p.max_region_size)
            for (const int i : region) disparity.data()[i] = kInvalidDisparity;
    }
}

StereoResult compute_disparity(ImageView<const std::uint8_t> left, ImageView<const std::uint8_t> right,
                               const StereoParams& params) {
    std::vector<ImageU8> pyr_l, pyr_r;  // pyr[i] is level i+1 (half size each)
    ImageView<const std::uint8_t> cur_l = left, cur_r = right;
    for (int i = 0; i < params.pyramid_levels; ++i) {
        pyr_l.push_back(downsample2(cur_l));
        pyr_r.push_back(downsample2(cur_r));
        cur_l = pyr_l.back().view();
        cur_r = pyr_r.back().view();
    }

    StereoResult result;
    ImageF32 disp = sgm_disparity(cur_l, cur_r, params.sgm, params.subpixel);
    if (params.pyramid_levels == 0) {
        result.confidence = ImageF32(disp.width(), disp.height(), 1.0f);
        for (std::size_t i = 0; i < disp.size(); ++i)
            if (!valid_disparity(disp.data()[i])) result.confidence.data()[i] = 0.0f;
    }
    for (int level = params.pyramid_levels - 1; level >= 0; --level) {
        const auto l = level == 0 ? left : pyr_l[static_cast<std::size_t>(level - 1)].view();
        const auto r = level == 0 ? right : pyr_r[static_cast<std::size_t>(level - 1)].view();
        ImageF32 finer, conf;
        refine_level(l, r, disp, params.refine, params.subpixel, finer, conf);
        disp = std::move(finer);
        result.confidence = std::move(conf);
    }
    remove_speckles(disp, params.speckle);
    for (std::size_t i = 0; i < disp.size(); ++i)
        if (!valid_disparity(disp.data()[i])) result.confidence.data()[i] = 0.0f;
    result.disparity = std::move(disp);
    return result;
}

}  // namespace einstar::depth
