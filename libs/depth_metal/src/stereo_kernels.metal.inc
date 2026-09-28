R"METAL(
#include <metal_stdlib>
using namespace metal;

// Mirrors libs/depth/src/stereo.cpp (the CPU reference). Integer stages are bit-exact.

constant float kInvalid = -1.0f;

struct Size2 { uint w; uint h; };

// ---- rectification + 2x2 box downsample (== CPU remap to full res, then downsample2) ----
struct RectifyArgs { uint src_w; uint src_h; uint full_w; uint full_h; uint out_w; uint out_h; };

inline float bilinear_u8(device const uchar* src, uint w, uint h, float sx, float sy) {
    const int x0 = int(floor(sx)), y0 = int(floor(sy));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= int(w) || y0 + 1 >= int(h)) return 0.0f;
    const float fx = sx - float(x0), fy = sy - float(y0);
    const float a = src[y0 * w + x0], b = src[y0 * w + x0 + 1];
    const float c = src[(y0 + 1) * w + x0], d = src[(y0 + 1) * w + x0 + 1];
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
}

kernel void rectify_half(device const uchar* src [[buffer(0)]], device const float2* map [[buffer(1)]],
                         device uchar* out [[buffer(2)]], constant RectifyArgs& a [[buffer(3)]],
                         uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.out_w || gid.y >= a.out_h) return;
    uint acc = 0;
    for (uint j = 0; j < 2; ++j)
        for (uint i = 0; i < 2; ++i) {
            const uint fx = 2 * gid.x + i, fy = 2 * gid.y + j;
            const float2 m = map[fy * a.full_w + fx];
            float v = 0.0f;
            const int x0 = int(floor(m.x)), y0 = int(floor(m.y));
            if (!(x0 < 0 || y0 < 0 || x0 + 1 >= int(a.src_w) || y0 + 1 >= int(a.src_h)))
                v = clamp(bilinear_u8(src, a.src_w, a.src_h, m.x, m.y) + 0.5f, 0.0f, 255.0f);
            acc += uint(v);
        }
    out[gid.y * a.out_w + gid.x] = uchar((acc + 2) / 4);
}

kernel void downsample2(device const uchar* src [[buffer(0)]], device uchar* out [[buffer(1)]],
                        constant Size2& s [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    const uint w = s.w / 2, h = s.h / 2;
    if (gid.x >= w || gid.y >= h) return;
    const uint r0 = 2 * gid.y * s.w, r1 = r0 + s.w;
    const uint x = 2 * gid.x;
    out[gid.y * w + gid.x] = uchar((uint(src[r0 + x]) + src[r0 + x + 1] + src[r1 + x] + src[r1 + x + 1] + 2) / 4);
}

// ---- census ----
struct CensusArgs { uint w; uint h; int rx; int ry; };

kernel void census(device const uchar* img [[buffer(0)]], device ulong* out [[buffer(1)]],
                   constant CensusArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const int x = int(gid.x), y = int(gid.y);
    ulong bits = 0;
    if (y >= a.ry && y < int(a.h) - a.ry && x >= a.rx && x < int(a.w) - a.rx) {
        const uchar c = img[y * a.w + x];
        for (int v = -a.ry; v <= a.ry; ++v)
            for (int u = -a.rx; u <= a.rx; ++u) {
                if (u == 0 && v == 0) continue;
                bits = (bits << 1) | ulong(img[(y + v) * int(a.w) + x + u] < c);
            }
    }
    out[gid.y * a.w + gid.x] = bits;
}

// ---- matching cost volume [y][x][k], u16 ----
struct CostArgs { uint w; uint h; uint nd; int min_disp; ushort invalid_cost; };

kernel void cost_volume(device const ulong* cl [[buffer(0)]], device const ulong* cr [[buffer(1)]],
                        device ushort* cost [[buffer(2)]], constant CostArgs& a [[buffer(3)]],
                        uint3 gid [[thread_position_in_grid]]) {
    const uint k = gid.x, x = gid.y, y = gid.z;
    if (k >= a.nd || x >= a.w || y >= a.h) return;
    const int xr = int(x) - (a.min_disp + int(k));
    ushort c = a.invalid_cost;
    if (xr >= 0) c = ushort(popcount(cl[y * a.w + x] ^ cr[y * a.w + uint(xr)]));
    cost[(ulong(y) * a.w + x) * a.nd + k] = c;
}

// ---- SGM path aggregation: one threadgroup per scanline, one thread per disparity ----
struct PathArgs {
    uint w; uint h; uint nd;
    int dx; int dy;
    int p1; int p2; uint adaptive; uint slant;
};

constant uint kMaxD = 512;

kernel void sgm_path(device const ushort* cost [[buffer(0)]], device ushort* sum [[buffer(1)]],
                     device const uchar* guide [[buffer(2)]], constant PathArgs& a [[buffer(3)]],
                     uint line [[threadgroup_position_in_grid]], uint k [[thread_index_in_threadgroup]],
                     uint tg_size [[threads_per_threadgroup]], uint lane [[thread_index_in_simdgroup]],
                     uint simd_id [[simdgroup_index_in_threadgroup]]) {
    threadgroup ushort prev[kMaxD];
    threadgroup ushort simd_mins[32];
    threadgroup ushort prev_min_shared;

    // Start pixel of this scanline (same enumeration as the CPU: x-entry border first, then y border).
    int x, y;
    const int W = int(a.w), H = int(a.h);
    if (a.dx != 0 && a.dy == 0) { x = a.dx > 0 ? 0 : W - 1; y = int(line); }
    else if (a.dx == 0) { x = int(line); y = a.dy > 0 ? 0 : H - 1; }
    else {
        // Diagonals: first H lines start on the x border, remaining on the y border (excluding the corner).
        if (int(line) < H) { x = a.dx > 0 ? 0 : W - 1; y = int(line); }
        else {
            int idx = int(line) - H;
            const int corner = a.dx > 0 ? 0 : W - 1;
            if (idx >= corner) idx += 1;
            x = idx; y = a.dy > 0 ? 0 : H - 1;
        }
    }
    const bool active = k < a.nd;

    ushort mine = active ? cost[(ulong(y) * a.w + uint(x)) * a.nd + k] : ushort(0xFFFF);
    if (active) {
        prev[k] = mine;
        const ulong si = (ulong(y) * a.w + uint(x)) * a.nd + k;
        sum[si] = ushort(sum[si] + mine);
    }
    // min reduction
    ushort m = simd_min(mine);
    if (lane == 0) simd_mins[simd_id] = m;
    threadgroup_barrier(mem_flags::mem_threadgroup);
    if (k == 0) {
        ushort mm = 0xFFFF;
        for (uint s = 0; s < (tg_size + 31) / 32; ++s) mm = min(mm, simd_mins[s]);
        prev_min_shared = mm;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    int prev_intensity = guide[uint(y) * a.w + uint(x)];
    x += a.dx;
    y += a.dy;
    while (x >= 0 && y >= 0 && x < W && y < H) {
        const int intensity = guide[uint(y) * a.w + uint(x)];
        int p2 = a.p2;
        if (a.adaptive) p2 = max(a.p1 + 1, a.p2 / (abs(intensity - prev_intensity) / 8 + 1));
        prev_intensity = intensity;
        const int pm = prev_min_shared;
        ushort cur = 0xFFFF;
        if (active) {
            const ushort c = cost[(ulong(y) * a.w + uint(x)) * a.nd + k];
            int best = min(int(prev[k]), pm + p2);
            if (k > 0) best = min(best, int(prev[k - 1]) + a.p1);
            if (k + 1 < a.nd) best = min(best, int(prev[k + 1]) + a.p1);
            if (a.slant) {
                if (k > 1) best = min(best, int(prev[k - 2]) + a.p1);
                if (k + 2 < a.nd) best = min(best, int(prev[k + 2]) + a.p1);
            }
            cur = ushort(int(c) + best - pm);
            const ulong si = (ulong(y) * a.w + uint(x)) * a.nd + k;
            sum[si] = ushort(sum[si] + cur);
        }
        ushort mn = simd_min(cur);
        threadgroup_barrier(mem_flags::mem_threadgroup);  // everyone has read prev
        if (active) prev[k] = cur;
        if (lane == 0) simd_mins[simd_id] = mn;
        threadgroup_barrier(mem_flags::mem_threadgroup);
        if (k == 0) {
            ushort mm = 0xFFFF;
            for (uint s = 0; s < (tg_size + 31) / 32; ++s) mm = min(mm, simd_mins[s]);
            prev_min_shared = mm;
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
        x += a.dx;
        y += a.dy;
    }
}

// ---- winner-takes-all (left) + right disparity from the same volume ----
struct WtaArgs { uint w; uint h; uint nd; int min_disp; float uniqueness; uint subpixel; };

inline float subpixel_offset(float cm, float c0, float cp) {
    const float denom = cm - 2.0f * c0 + cp;
    if (denom <= 0.0f) return 0.0f;
    return clamp(0.5f * (cm - cp) / denom, -0.5f, 0.5f);
}

kernel void wta_left(device const ushort* sum [[buffer(0)]], device float* disp [[buffer(1)]],
                     constant WtaArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const int x = int(gid.x);
    device const ushort* s = sum + (ulong(gid.y) * a.w + gid.x) * a.nd;
    const int max_k = min(int(a.nd), x - a.min_disp + 1);
    float out = kInvalid;
    if (max_k > 0) {
        int best_k = 0;
        for (int k = 1; k < max_k; ++k) if (s[k] < s[best_k]) best_k = k;
        ushort second = 0xFFFF;
        for (int k = 0; k < max_k; ++k) if (abs(k - best_k) > 1) second = min(second, s[k]);
        if (!(second != 0xFFFF && float(s[best_k]) >= a.uniqueness * float(second))) {
            float d = float(a.min_disp + best_k);
            if (a.subpixel && best_k > 0 && best_k + 1 < max_k) d += subpixel_offset(s[best_k - 1], s[best_k], s[best_k + 1]);
            out = d;
        }
    }
    disp[gid.y * a.w + gid.x] = out;
}

kernel void wta_right(device const ushort* sum [[buffer(0)]], device int* rdisp [[buffer(1)]],
                      constant WtaArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const int xr = int(gid.x);
    int best_k = -1;
    ushort best = 0xFFFF;
    for (int k = 0; k < int(a.nd); ++k) {
        const int xl = xr + a.min_disp + k;
        if (xl >= int(a.w)) break;
        const ushort v = sum[(ulong(gid.y) * a.w + uint(xl)) * a.nd + uint(k)];
        if (v < best) { best = v; best_k = k; }
    }
    rdisp[gid.y * a.w + gid.x] = best_k < 0 ? -1 : a.min_disp + best_k;
}

struct LrArgs { uint w; uint h; float max_diff; };

kernel void lr_check(device float* disp [[buffer(0)]], device const int* rdisp [[buffer(1)]],
                     constant LrArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    const float d = disp[i];
    if (d < 0) return;
    const int xr = int(rint(float(gid.x) - d));
    if (xr < 0 || xr >= int(a.w) || rdisp[gid.y * a.w + uint(xr)] < 0 || abs(float(rdisp[gid.y * a.w + uint(xr)]) - d) > a.max_diff)
        disp[i] = kInvalid;
}

// ---- 3x3 median of valid neighbours ----
kernel void median3(device const float* in [[buffer(0)]], device float* out [[buffer(1)]],
                    constant Size2& s [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= s.w || gid.y >= s.h) return;
    const int x = int(gid.x), y = int(gid.y);
    const uint i = gid.y * s.w + gid.x;
    if (in[i] < 0) { out[i] = kInvalid; return; }
    float buf[9];
    int n = 0;
    for (int v = -1; v <= 1; ++v)
        for (int u = -1; u <= 1; ++u) {
            const int xx = x + u, yy = y + v;
            if (xx < 0 || yy < 0 || xx >= int(s.w) || yy >= int(s.h)) continue;
            const float d = in[yy * int(s.w) + xx];
            if (d < 0) continue;
            buf[n++] = d;
        }
    // insertion sort; the CPU uses nth_element on the same set, so the n/2-th element matches
    for (int i2 = 1; i2 < n; ++i2) {
        const float t = buf[i2];
        int j = i2 - 1;
        while (j >= 0 && buf[j] > t) { buf[j + 1] = buf[j]; --j; }
        buf[j + 1] = t;
    }
    out[i] = buf[n / 2];
}

// ---- slanted-window ZNCC refinement to the next finer level ----
struct RefineArgs { uint w; uint h; uint cw; uint ch; int radius; int search_radius; float min_zncc; uint subpixel; };

inline float zncc_slanted(device const uchar* a, device const uchar* b, int W, int x, int y, float d, float gx, float gy, int r) {
    float sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    const int n = (2 * r + 1) * (2 * r + 1);
    for (int v = -r; v <= r; ++v) {
        device const uchar* ra = a + (y + v) * W;
        device const uchar* rb = b + (y + v) * W;
        for (int u = -r; u <= r; ++u) {
            const float xb = float(x + u) - (d + gx * float(u) + gy * float(v));
            const int x0 = int(floor(xb));
            if (x0 < 0 || x0 + 1 >= W) return -2.0f;
            const float f = xb - float(x0);
            const float ib = float(rb[x0]) * (1.0f - f) + float(rb[x0 + 1]) * f;
            const float ia = ra[x + u];
            sa += ia; sb += ib; saa += ia * ia; sbb += ib * ib; sab += ia * ib;
        }
    }
    const float va = saa - sa * sa / n;
    const float vb = sbb - sb * sb / n;
    if (va < 1e-6f || vb < 1e-6f) return -1.0f;
    return (sab - sa * sb / n) / sqrt(va * vb);
}

kernel void refine_slanted(device const uchar* left [[buffer(0)]], device const uchar* right [[buffer(1)]],
                           device const float* coarse [[buffer(2)]], device float* out_disp [[buffer(3)]],
                           device float* out_conf [[buffer(4)]], constant RefineArgs& a [[buffer(5)]],
                           uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const int x = int(gid.x), y = int(gid.y), r = a.radius;
    const uint i = gid.y * a.w + gid.x;
    out_disp[i] = kInvalid;
    out_conf[i] = 0.0f;
    if (y < r || y >= int(a.h) - r || x < r || x >= int(a.w) - r) return;
    const int cw = int(a.cw), ch = int(a.ch);
    const int cx = min(x / 2, cw - 1), cy = min(y / 2, ch - 1);
    const float d0 = coarse[cy * cw + cx];
    if (d0 < 0.0f) return;
    auto C = [&](int xx, int yy) { return coarse[yy * cw + xx]; };
    float gx = 0, gy = 0;
    const float c = d0;
    if (cx > 0 && cx + 1 < cw && C(cx - 1, cy) >= 0 && C(cx + 1, cy) >= 0) {
        const float g = 0.5f * (C(cx + 1, cy) - C(cx - 1, cy));
        if (abs(C(cx + 1, cy) - c) < 2 && abs(c - C(cx - 1, cy)) < 2) gx = g;
    }
    if (cy > 0 && cy + 1 < ch && C(cx, cy - 1) >= 0 && C(cx, cy + 1) >= 0) {
        const float g = 0.5f * (C(cx, cy + 1) - C(cx, cy - 1));
        if (abs(C(cx, cy + 1) - c) < 2 && abs(c - C(cx, cy - 1)) < 2) gy = g;
    }
    gx = clamp(gx, -0.9f, 0.9f);
    gy = clamp(gy, -0.9f, 0.9f);
    const float step = 0.5f;
    const float center = 2.0f * d0;
    const int nd = min(127, int(2 * a.search_radius / step) + 1);
    const float lo = center - float(a.search_radius);
    float scores[128];
    float best = -2.0f;
    int best_k = -1;
    for (int k = 0; k < nd; ++k) {
        const float d = lo + step * float(k);
        scores[k] = d < 0 ? -2.0f : zncc_slanted(left, right, int(a.w), x, y, d, gx, gy, r);
        if (scores[k] > best) { best = scores[k]; best_k = k; }
    }
    if (best_k < 0 || best < a.min_zncc) return;
    float d = lo + step * float(best_k);
    if (a.subpixel && best_k > 0 && best_k + 1 < nd && scores[best_k - 1] > -2.0f && scores[best_k + 1] > -2.0f)
        d += step * subpixel_offset(1.0f - scores[best_k - 1], 1.0f - best, 1.0f - scores[best_k + 1]);
    out_disp[i] = d;
    out_conf[i] = best;
}
)METAL"
R"METAL(
// ---- speckle removal: union-find connected components (4-connectivity, |d_a - d_b| <= max_diff) ----
struct CclArgs { uint w; uint h; float max_diff; uint min_size; };

inline int ccl_find(device atomic_int* L, int x) {
    int p = atomic_load_explicit(&L[x], memory_order_relaxed);
    while (p != x) {
        x = p;
        p = atomic_load_explicit(&L[x], memory_order_relaxed);
    }
    return x;
}

inline void ccl_unite(device atomic_int* L, int a, int b) {
    bool done = false;
    while (!done) {
        a = ccl_find(L, a);
        b = ccl_find(L, b);
        if (a < b) {
            const int old = atomic_fetch_min_explicit(&L[b], a, memory_order_relaxed);
            done = old == b;
            b = old;
        } else if (b < a) {
            const int old = atomic_fetch_min_explicit(&L[a], b, memory_order_relaxed);
            done = old == a;
            a = old;
        } else {
            done = true;
        }
    }
}

kernel void ccl_init(device const float* disp [[buffer(0)]], device atomic_int* L [[buffer(1)]],
                     device atomic_uint* sizes [[buffer(2)]], constant CclArgs& a [[buffer(3)]],
                     uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    atomic_store_explicit(&L[i], disp[i] >= 0 ? int(i) : -1, memory_order_relaxed);
    atomic_store_explicit(&sizes[i], 0u, memory_order_relaxed);
}

kernel void ccl_merge(device const float* disp [[buffer(0)]], device atomic_int* L [[buffer(1)]],
                      constant CclArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    const float d = disp[i];
    if (d < 0) return;
    if (gid.x + 1 < a.w) {
        const float dr = disp[i + 1];
        if (dr >= 0 && abs(dr - d) <= a.max_diff) ccl_unite(L, int(i), int(i + 1));
    }
    if (gid.y + 1 < a.h) {
        const float dd = disp[i + a.w];
        if (dd >= 0 && abs(dd - d) <= a.max_diff) ccl_unite(L, int(i), int(i + a.w));
    }
}

kernel void ccl_count(device const float* disp [[buffer(0)]], device atomic_int* L [[buffer(1)]],
                      device atomic_uint* sizes [[buffer(2)]], constant CclArgs& a [[buffer(3)]],
                      uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    if (disp[i] < 0) return;
    const int root = ccl_find(L, int(i));
    atomic_fetch_add_explicit(&sizes[root], 1u, memory_order_relaxed);
}

kernel void ccl_filter(device float* disp [[buffer(0)]], device float* conf [[buffer(1)]], device atomic_int* L [[buffer(2)]],
                       device const atomic_uint* sizes [[buffer(3)]], constant CclArgs& a [[buffer(4)]],
                       uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    if (disp[i] < 0) { conf[i] = 0.0f; return; }
    const int root = ccl_find(L, int(i));
    if (atomic_load_explicit(&sizes[root], memory_order_relaxed) < a.min_size) {
        disp[i] = kInvalid;
        conf[i] = 0.0f;
    }
}

// ---- disparity -> organised points, normals, weights (mirrors depth::disparity_to_points) ----
struct PointsArgs { uint w; uint h; float f; float cx; float cy; float baseline; float min_depth; float max_depth; float max_jump; };

kernel void disparity_points(device const float* disp [[buffer(0)]], device const float* conf [[buffer(1)]],
                             device float4* pts [[buffer(2)]], constant PointsArgs& a [[buffer(3)]],
                             uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    float4 p = float4(0);
    const float d = disp[i];
    if (d > 0) {
        const float z = a.f * a.baseline / d;
        if (z >= a.min_depth && z <= a.max_depth)
            p = float4((float(gid.x) - a.cx) * z / a.f, (float(gid.y) - a.cy) * z / a.f, z, 1.0f);
    }
    pts[i] = p;
}

kernel void point_normals(device const float4* pts [[buffer(0)]], device const float* conf [[buffer(1)]],
                          device float4* nrm [[buffer(2)]], device float* wts [[buffer(3)]], constant PointsArgs& a [[buffer(4)]],
                          uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    const float4 p = pts[i];
    float w = p.w > 0 ? conf[i] : 0.0f;
    float4 n = float4(0);
    if (p.w > 0 && gid.x > 0 && gid.y > 0 && gid.x + 1 < a.w && gid.y + 1 < a.h) {
        const float4 l = pts[i - 1], r = pts[i + 1], t = pts[i - a.w], b = pts[i + a.w];
        if (l.w > 0 && r.w > 0 && t.w > 0 && b.w > 0 && abs(l.z - r.z) <= a.max_jump && abs(t.z - b.z) <= a.max_jump) {
            float3 nn = cross(r.xyz - l.xyz, b.xyz - t.xyz);
            const float len = length(nn);
            if (len >= 1e-9f) {
                nn /= len;
                if (dot(nn, p.xyz) > 0) nn = -nn;
                n = float4(nn, 0);
                w *= max(0.0f, -dot(nn, normalize(p.xyz)));
            }
        }
    }
    nrm[i] = n;
    wts[i] = w;
}
)METAL"
