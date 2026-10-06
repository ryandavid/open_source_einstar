R"METAL(
#include <metal_stdlib>
using namespace metal;

// Mirrors libs/depth/src/stereo.cpp (the CPU reference). Integer stages are bit-exact.

// Disparities are signed (the rectified right camera's principal point is shifted), so "no match" is a
// value no disparity reaches; integer right disparities use kNoMatch.
constant float kInvalid = -1.0e9f;
constant int kNoMatch = -2147483647 - 1;
inline bool valid(float d) { return d > -1.0e8f; }

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

// Full-resolution rectified image (== CPU remap); downsample2 then gives exactly rectify_half's output.
kernel void rectify_full(device const uchar* src [[buffer(0)]], device const float2* map [[buffer(1)]],
                         device uchar* out [[buffer(2)]], constant RectifyArgs& a [[buffer(3)]],
                         uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.full_w || gid.y >= a.full_h) return;
    const float2 m = map[gid.y * a.full_w + gid.x];
    float v = 0.0f;
    const int x0 = int(floor(m.x)), y0 = int(floor(m.y));
    if (!(x0 < 0 || y0 < 0 || x0 + 1 >= int(a.src_w) || y0 + 1 >= int(a.src_h)))
        v = clamp(bilinear_u8(src, a.src_w, a.src_h, m.x, m.y) + 0.5f, 0.0f, 255.0f);
    out[gid.y * a.full_w + gid.x] = uchar(uint(v));
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

// Census costs are at most 25 (5x5) and every path value at most cost + P2, so both fit a byte:
// half the memory traffic of the (bandwidth-bound) aggregation, same integers.
kernel void cost_volume(device const ulong* cl [[buffer(0)]], device const ulong* cr [[buffer(1)]],
                        device uchar* cost [[buffer(2)]], constant CostArgs& a [[buffer(3)]],
                        uint3 gid [[thread_position_in_grid]]) {
    const uint k = gid.x, x = gid.y, y = gid.z;
    if (k >= a.nd || x >= a.w || y >= a.h) return;
    const int xr = int(x) - (a.min_disp + int(k));
    uint c = a.invalid_cost;
    if (xr >= 0 && xr < int(a.w)) c = popcount(cl[y * a.w + x] ^ cr[y * a.w + uint(xr)]);
    cost[(ulong(y) * a.w + x) * a.nd + k] = uchar(c);
}

// ---- SGM path aggregation: one threadgroup per scanline, one thread per disparity ----
// (Measured alternatives: one SIMD group per line with disparity blocks per lane, double-buffered
// single-barrier steps, and one-step-ahead prefetch were all no faster or slower on Apple GPUs; the
// kernel is bound by its arithmetic volume.)
struct PathArgs {
    uint w; uint h; uint nd;
    int dx; int dy;
    int p1; int p2; uint adaptive; uint slant;
    uint first;  // the first direction writes the sums (no clearing pass, no read)
    // Adaptive P2 by intensity step |dI| / 8 (0..31): max(P1 + 1, P2 / (|dI| / 8 + 1)), divided on the host.
    uint p2_by_step[32];
};

constant uint kMaxD = 512;

kernel void sgm_path(device const uchar* cost [[buffer(0)]], device ushort* sum [[buffer(1)]],
                     device const uchar* guide [[buffer(2)]], constant PathArgs& a [[buffer(3)]],
                     uint line [[threadgroup_position_in_grid]], uint k [[thread_index_in_threadgroup]],
                     uint lane [[thread_index_in_simdgroup]], uint simd_id [[simdgroup_index_in_threadgroup]],
                     uint nsimd [[simdgroups_per_threadgroup]]) {
    threadgroup ushort prev[kMaxD];
    // One slot per SIMD group. The width is the GPU's (32 on Apple, 64 on AMD, 8-32 on Intel), so
    // size for the narrowest and read back only the groups that exist.
    threadgroup ushort simd_mins[kMaxD / 8];
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

    ushort mine = active ? ushort(cost[(ulong(y) * a.w + uint(x)) * a.nd + k]) : ushort(0xFFFF);
    if (active) {
        prev[k] = mine;
        const ulong si = (ulong(y) * a.w + uint(x)) * a.nd + k;
        sum[si] = a.first ? mine : ushort(sum[si] + mine);
    }
    // min reduction
    ushort m = simd_min(mine);
    if (lane == 0) simd_mins[simd_id] = m;
    threadgroup_barrier(mem_flags::mem_threadgroup);
    if (k == 0) {
        ushort mm = 0xFFFF;
        for (uint s = 0; s < nsimd; ++s) mm = min(mm, simd_mins[s]);
        prev_min_shared = mm;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    int prev_intensity = guide[uint(y) * a.w + uint(x)];
    x += a.dx;
    y += a.dy;
    while (x >= 0 && y >= 0 && x < W && y < H) {
        const int intensity = guide[uint(y) * a.w + uint(x)];
        const int p2 = a.adaptive ? int(a.p2_by_step[abs(intensity - prev_intensity) / 8]) : a.p2;
        prev_intensity = intensity;
        const int pm = prev_min_shared;
        ushort cur = 0xFFFF;
        if (active) {
            const ushort c = ushort(cost[(ulong(y) * a.w + uint(x)) * a.nd + k]);
            int best = min(int(prev[k]), pm + p2);
            if (k > 0) best = min(best, int(prev[k - 1]) + a.p1);
            if (k + 1 < a.nd) best = min(best, int(prev[k + 1]) + a.p1);
            if (a.slant) {
                if (k > 1) best = min(best, int(prev[k - 2]) + a.p1);
                if (k + 2 < a.nd) best = min(best, int(prev[k + 2]) + a.p1);
            }
            cur = ushort(int(c) + best - pm);
            const ulong si = (ulong(y) * a.w + uint(x)) * a.nd + k;
            sum[si] = a.first ? cur : ushort(sum[si] + cur);
        }
        ushort mn = simd_min(cur);
        threadgroup_barrier(mem_flags::mem_threadgroup);  // everyone has read prev
        if (active) prev[k] = cur;
        if (lane == 0) simd_mins[simd_id] = mn;
        threadgroup_barrier(mem_flags::mem_threadgroup);
        if (k == 0) {
            ushort mm = 0xFFFF;
            for (uint s = 0; s < nsimd; ++s) mm = min(mm, simd_mins[s]);
            prev_min_shared = mm;
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
        x += a.dx;
        y += a.dy;
    }
}

// Barrier-free variant for GPUs where the threadgroup barriers above dominate (AMD: 3 barriers per
// step across 4 wavefronts). One SIMD group per scanline; lane l holds disparities l*DPL .. l*DPL+DPL-1
// and gets its neighbours' edge values by shuffles. Same integer arithmetic, so the same sums.
template <int DPL>
inline void sgm_path_simd_body(device const uchar* cost, device ushort* sum, device const uchar* guide, constant PathArgs& a,
                               uint line, uint lane) {
    int x, y;
    const int W = int(a.w), H = int(a.h);
    if (a.dx != 0 && a.dy == 0) { x = a.dx > 0 ? 0 : W - 1; y = int(line); }
    else if (a.dx == 0) { x = int(line); y = a.dy > 0 ? 0 : H - 1; }
    else {
        if (int(line) < H) { x = a.dx > 0 ? 0 : W - 1; y = int(line); }
        else {
            int idx = int(line) - H;
            const int corner = a.dx > 0 ? 0 : W - 1;
            if (idx >= corner) idx += 1;
            x = idx; y = a.dy > 0 ? 0 : H - 1;
        }
    }
    const int nd = int(a.nd);
    const int k0 = int(lane) * DPL;
    uint prev[DPL];
    uint lmin = 0xFFFF;
    {
        const ulong base = (ulong(y) * a.w + uint(x)) * a.nd;
        for (int j = 0; j < DPL; ++j) {
            const int k = k0 + j;
            const uint v = k < nd ? uint(cost[base + uint(k)]) : 0xFFFFu;
            prev[j] = v;
            if (k < nd) sum[base + uint(k)] = a.first ? ushort(v) : ushort(sum[base + uint(k)] + v);
            lmin = min(lmin, v);
        }
    }
    int pm = int(simd_min(lmin));
    int prev_intensity = guide[uint(y) * a.w + uint(x)];
    x += a.dx;
    y += a.dy;
    while (x >= 0 && y >= 0 && x < W && y < H) {
        const int intensity = guide[uint(y) * a.w + uint(x)];
        const int p2 = a.adaptive ? int(a.p2_by_step[abs(intensity - prev_intensity) / 8]) : a.p2;
        prev_intensity = intensity;
        // Neighbouring lanes' edge values (only read where k - 1, k - 2, k + 1, k + 2 are in range).
        const uint l1 = simd_shuffle_up(prev[DPL - 1], 1), l2 = simd_shuffle_up(prev[DPL - 2], 1);
        const uint r1 = simd_shuffle_down(prev[0], 1), r2 = simd_shuffle_down(prev[1], 1);
        const ulong base = (ulong(y) * a.w + uint(x)) * a.nd;
        uint cur[DPL];
        lmin = 0xFFFF;
        for (int j = 0; j < DPL; ++j) {
            const int k = k0 + j;
            if (k >= nd) { cur[j] = 0xFFFF; continue; }
            const int c = int(cost[base + uint(k)]);
            const int km1 = j >= 1 ? int(prev[j - 1]) : int(l1);
            const int km2 = j >= 2 ? int(prev[j - 2]) : (j == 1 ? int(l1) : int(l2));
            const int kp1 = j + 1 < DPL ? int(prev[j + 1]) : int(r1);
            const int kp2 = j + 2 < DPL ? int(prev[j + 2]) : (j + 1 < DPL ? int(r1) : int(r2));
            int best = min(int(prev[j]), pm + p2);
            if (k > 0) best = min(best, km1 + a.p1);
            if (k + 1 < nd) best = min(best, kp1 + a.p1);
            if (a.slant) {
                if (k > 1) best = min(best, km2 + a.p1);
                if (k + 2 < nd) best = min(best, kp2 + a.p1);
            }
            const uint v = uint(ushort(c + best - pm));
            cur[j] = v;
            sum[base + uint(k)] = a.first ? ushort(v) : ushort(sum[base + uint(k)] + v);
            lmin = min(lmin, v);
        }
        for (int j = 0; j < DPL; ++j) prev[j] = cur[j];
        pm = int(simd_min(lmin));
        x += a.dx;
        y += a.dy;
    }
}

#define SGM_PATH_SIMD(DPL)                                                                                       \
    kernel void sgm_path_simd##DPL(device const uchar* cost [[buffer(0)]], device ushort* sum [[buffer(1)]],     \
                                   device const uchar* guide [[buffer(2)]], constant PathArgs& a [[buffer(3)]], \
                                   uint line [[threadgroup_position_in_grid]], uint lane [[thread_index_in_simdgroup]]) { \
        sgm_path_simd_body<DPL>(cost, sum, guide, a, line, lane);                                                \
    }
SGM_PATH_SIMD(2)
SGM_PATH_SIMD(4)
SGM_PATH_SIMD(8)

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
    // Candidates whose right pixel x - (min_disp + k) lies inside the image.
    const int min_k = max(0, x - a.min_disp - (int(a.w) - 1));
    const int max_k = min(int(a.nd), x - a.min_disp + 1);
    float out = kInvalid;
    if (max_k > min_k) {
        int best_k = min_k;
        for (int k = min_k + 1; k < max_k; ++k) if (s[k] < s[best_k]) best_k = k;
        ushort second = 0xFFFF;
        for (int k = min_k; k < max_k; ++k) if (abs(k - best_k) > 1) second = min(second, s[k]);
        if (!(second != 0xFFFF && float(s[best_k]) >= a.uniqueness * float(second))) {
            float d = float(a.min_disp + best_k);
            if (a.subpixel && best_k > min_k && best_k + 1 < max_k) d += subpixel_offset(s[best_k - 1], s[best_k], s[best_k + 1]);
            out = d;
        }
    }
    disp[gid.y * a.w + gid.x] = out;
}

// Both WTA passes for one image row per threadgroup, for GPUs whose memory system does not absorb the
// strided per-pixel scans above (AMD: 40 -> 0.7 ms). One SIMD group per left pixel, lanes over the
// disparities, so every read is contiguous; the right disparities come from the same reads as a
// per-row minimum of packed (cost << 16 | k) keys in threadgroup memory. Results are identical to
// wta_left + wta_right: ties go to the lowest k, and all-0xFFFF right candidates stay invalid.
constant uint kWtaMaxRow = 1024;

kernel void wta_rows(device const ushort* sum [[buffer(0)]], device float* disp [[buffer(1)]], device int* rdisp [[buffer(2)]],
                     constant WtaArgs& a [[buffer(3)]], uint y [[threadgroup_position_in_grid]],
                     uint tid [[thread_index_in_threadgroup]], uint tg_size [[threads_per_threadgroup]],
                     uint lane [[thread_index_in_simdgroup]], uint width [[threads_per_simdgroup]],
                     uint simd_id [[simdgroup_index_in_threadgroup]], uint nsimd [[simdgroups_per_threadgroup]]) {
    threadgroup atomic_uint right_best[kWtaMaxRow];
    for (uint x = tid; x < a.w; x += tg_size) atomic_store_explicit(&right_best[x], 0xFFFFFFFFu, memory_order_relaxed);
    threadgroup_barrier(mem_flags::mem_threadgroup);
    for (uint x = simd_id; x < a.w; x += nsimd) {
        device const ushort* s = sum + (ulong(y) * a.w + x) * a.nd;
        const int min_k = max(0, int(x) - a.min_disp - (int(a.w) - 1));
        const int max_k = min(int(a.nd), int(x) - a.min_disp + 1);
        if (max_k <= min_k) {  // uniform across the SIMD group
            if (lane == 0) disp[y * a.w + x] = kInvalid;
            continue;
        }
        uint best = 0xFFFFFFFFu;
        for (int k = min_k + int(lane); k < max_k; k += int(width)) {
            const uint v = s[k];
            const uint key = (v << 16) | uint(k);
            best = min(best, key);
            // The right pixel that sees this candidate: xr = x - min_disp - k (inside the row for min_k <= k < max_k).
            if (v != 0xFFFFu) atomic_fetch_min_explicit(&right_best[uint(int(x) - a.min_disp - k)], key, memory_order_relaxed);
        }
        best = simd_min(best);
        const int best_k = int(best & 0xFFFFu);
        ushort second = 0xFFFF;
        for (int k = min_k + int(lane); k < max_k; k += int(width))
            if (abs(k - best_k) > 1) second = min(second, s[k]);
        second = simd_min(second);
        if (lane == 0) {
            float out = kInvalid;
            if (!(second != 0xFFFF && float(best >> 16) >= a.uniqueness * float(second))) {
                float d = float(a.min_disp + best_k);
                if (a.subpixel && best_k > min_k && best_k + 1 < max_k) d += subpixel_offset(s[best_k - 1], s[best_k], s[best_k + 1]);
                out = d;
            }
            disp[y * a.w + x] = out;
        }
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    for (uint x = tid; x < a.w; x += tg_size) {
        const uint key = atomic_load_explicit(&right_best[x], memory_order_relaxed);
        rdisp[y * a.w + x] = key == 0xFFFFFFFFu ? kNoMatch : a.min_disp + int(key & 0xFFFFu);
    }
}

kernel void wta_right(device const ushort* sum [[buffer(0)]], device int* rdisp [[buffer(1)]],
                      constant WtaArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const int xr = int(gid.x);
    int best_k = -1;
    ushort best = 0xFFFF;
    for (int k = max(0, -(xr + a.min_disp)); k < int(a.nd); ++k) {
        const int xl = xr + a.min_disp + k;
        if (xl >= int(a.w)) break;
        const ushort v = sum[(ulong(gid.y) * a.w + uint(xl)) * a.nd + uint(k)];
        if (v < best) { best = v; best_k = k; }
    }
    rdisp[gid.y * a.w + gid.x] = best_k < 0 ? kNoMatch : a.min_disp + best_k;
}

struct LrArgs { uint w; uint h; float max_diff; };

kernel void lr_check(device float* disp [[buffer(0)]], device const int* rdisp [[buffer(1)]],
                     constant LrArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    const float d = disp[i];
    if (!valid(d)) return;
    const int xr = int(rint(float(gid.x) - d));
    if (xr < 0 || xr >= int(a.w) || rdisp[gid.y * a.w + uint(xr)] == kNoMatch || abs(float(rdisp[gid.y * a.w + uint(xr)]) - d) > a.max_diff)
        disp[i] = kInvalid;
}

// ---- 3x3 median of valid neighbours ----
kernel void median3(device const float* in [[buffer(0)]], device float* out [[buffer(1)]],
                    constant Size2& s [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= s.w || gid.y >= s.h) return;
    const int x = int(gid.x), y = int(gid.y);
    const uint i = gid.y * s.w + gid.x;
    if (!valid(in[i])) { out[i] = kInvalid; return; }
    float buf[9];
    int n = 0;
    for (int v = -1; v <= 1; ++v)
        for (int u = -1; u <= 1; ++u) {
            const int xx = x + u, yy = y + v;
            if (xx < 0 || yy < 0 || xx >= int(s.w) || yy >= int(s.h)) continue;
            const float d = in[yy * int(s.w) + xx];
            if (!valid(d)) continue;
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
// Number of disparity candidates (2 * search_radius / 0.5 + 1), specialised at pipeline creation so the
// per-candidate accumulators unroll into registers.
constant uint kRefineCandidates [[function_constant(0)]];
constant int kMaxRefineCandidates = 17;

struct RefineArgs { uint w; uint h; uint cw; uint ch; int radius; int search_radius; float min_zncc; uint subpixel; uint reject_edge; };

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
    if (!valid(d0)) return;
    auto C = [&](int xx, int yy) { return coarse[yy * cw + xx]; };
    float gx = 0, gy = 0;
    const float c = d0;
    if (cx > 0 && cx + 1 < cw && valid(C(cx - 1, cy)) && valid(C(cx + 1, cy))) {
        const float g = 0.5f * (C(cx + 1, cy) - C(cx - 1, cy));
        if (abs(C(cx + 1, cy) - c) < 2 && abs(c - C(cx - 1, cy)) < 2) gx = g;
    }
    if (cy > 0 && cy + 1 < ch && valid(C(cx, cy - 1)) && valid(C(cx, cy + 1))) {
        const float g = 0.5f * (C(cx, cy + 1) - C(cx, cy - 1));
        if (abs(C(cx, cy + 1) - c) < 2 && abs(c - C(cx, cy - 1)) < 2) gy = g;
    }
    gx = clamp(gx, -0.9f, 0.9f);
    gy = clamp(gy, -0.9f, 0.9f);
    const float step = 0.5f;
    const float center = 2.0f * d0;
    const float lo = center - float(a.search_radius);
    // All candidates in one pass over the window (each candidate accumulates in the same order as a
    // per-candidate loop, so the scores are identical).
    const int ncand = int(kRefineCandidates);
    float sa = 0, saa = 0, sb[kMaxRefineCandidates], sbb[kMaxRefineCandidates], sab[kMaxRefineCandidates];
    uint dead = 0;  // candidates whose window leaves the image
    for (int k = 0; k < ncand; ++k) { sb[k] = 0; sbb[k] = 0; sab[k] = 0; }
    const int W = int(a.w);
    for (int v = -r; v <= r; ++v) {
        device const uchar* ra = left + (y + v) * W;
        device const uchar* rb = right + (y + v) * W;
        for (int u = -r; u <= r; ++u) {
            const float ia = ra[x + u];
            sa += ia; saa += ia * ia;
            for (int k = 0; k < ncand; ++k) {
                const float d = lo + step * float(k);
                const float xb = float(x + u) - (d + gx * float(u) + gy * float(v));
                const int x0 = int(floor(xb));
                if (x0 < 0 || x0 + 1 >= W) { dead |= 1u << k; continue; }
                const float f = xb - float(x0);
                const float ib = float(rb[x0]) * (1.0f - f) + float(rb[x0 + 1]) * f;
                sb[k] += ib; sbb[k] += ib * ib; sab[k] += ia * ib;
            }
        }
    }
    const float n = float((2 * r + 1) * (2 * r + 1));
    const float va = saa - sa * sa / n;
    // Only the best score and its two neighbours are needed (for the subpixel fit).
    float best = -2.0f, best_prev = -2.0f, best_next = -2.0f, prev = -2.0f;
    int best_k = -1;
    for (int k = 0; k < ncand; ++k) {
        float sc = -2.0f;
        if (!(dead & (1u << k))) {
            const float vb = sbb[k] - sb[k] * sb[k] / n;
            sc = (va < 1e-6f || vb < 1e-6f) ? -1.0f : (sab[k] - sa * sb[k] / n) / sqrt(va * vb);
        }
        if (k == best_k + 1) best_next = sc;
        if (sc > best) { best = sc; best_k = k; best_prev = prev; best_next = -2.0f; }
        prev = sc;
    }
    if (best_k < 0 || best < a.min_zncc) return;
    if (a.reject_edge && (best_k == 0 || best_k == ncand - 1)) return;
    float d = lo + step * float(best_k);
    if (a.subpixel && best_k > 0 && best_k + 1 < ncand && best_prev > -2.0f && best_next > -2.0f)
        d += step * subpixel_offset(1.0f - best_prev, 1.0f - best, 1.0f - best_next);
    out_disp[i] = d;
    out_conf[i] = best;
}

// ---- detail refinement on the full-resolution images (mirrors depth::refine_detail) ----
struct DetailArgs { uint w; uint h; uint fw; uint fh; int radius; int K; float step; float min_zncc; uint subpixel; uint enabled; };
// 2 * K + 1, specialised at pipeline creation like kRefineCandidates (accumulators in registers).
constant uint kDetailCandidates [[function_constant(1)]];
constant int kMaxDetailCandidates = 17;

inline float detail_grad(float l, float c, float r) {
    if (!valid(l) || !valid(r) || abs(r - c) >= 1.0f || abs(c - l) >= 1.0f) return 0.0f;
    return clamp(0.5f * (r - l), -0.9f, 0.9f);
}

kernel void refine_detail(device const uchar* left [[buffer(0)]], device const uchar* right [[buffer(1)]],
                          device const float* first [[buffer(2)]], device float* out [[buffer(3)]],
                          device atomic_int* claims [[buffer(4)]], constant DetailArgs& a [[buffer(5)]],
                          uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const int x = int(gid.x), y = int(gid.y), W = int(a.w), H = int(a.h);
    const uint i = gid.y * a.w + gid.x;
    atomic_store_explicit(&claims[i], kNoMatch, memory_order_relaxed);  // for occlusion_claim (next dispatch)
    const float d = first[i];
    out[i] = d;
    if (!a.enabled || !valid(d) || x < 1 || y < 1 || x + 1 >= W || y + 1 >= H) return;
    const int n = 2 * a.radius;
    const int x0 = 2 * x - a.radius + 1, y0 = 2 * y - a.radius + 1;
    if (x0 < 0 || y0 < 0 || x0 + n > int(a.fw) || y0 + n > int(a.fh)) return;
    const float gx = detail_grad(first[i - 1], d, first[i + 1]), gy = detail_grad(first[i - a.w], d, first[i + a.w]);
    const float cx = 2.0f * float(x) + 0.5f, cy = 2.0f * float(y) + 0.5f;
    const float D0 = 2.0f * d;
    const int nc = int(kDetailCandidates);
    float sa = 0, saa = 0, sb[kMaxDetailCandidates], sbb[kMaxDetailCandidates], sab[kMaxDetailCandidates];
    uint dead = 0;
    for (int k = 0; k < nc; ++k) { sb[k] = 0; sbb[k] = 0; sab[k] = 0; }
    const int FW = int(a.fw);
    for (int j = 0; j < n; ++j) {
        const int Y = y0 + j;
        device const uchar* ra = left + Y * FW;
        device const uchar* rb = right + Y * FW;
        const float dy = gy * (float(Y) - cy);
        for (int u = 0; u < n; ++u) {
            const int X = x0 + u;
            const float ia = ra[X];
            sa += ia; saa += ia * ia;
            const float base = float(X) - (D0 + gx * (float(X) - cx) + dy);
            for (int k = 0; k < nc; ++k) {
                const float xb = base - a.step * float(k - a.K);
                const int xi = int(floor(xb));
                if (xi < 0 || xi + 1 >= FW) { dead |= 1u << k; continue; }
                const float f = xb - float(xi);
                const float ib = float(rb[xi]) * (1.0f - f) + float(rb[xi + 1]) * f;
                sb[k] += ib; sbb[k] += ib * ib; sab[k] += ia * ib;
            }
        }
    }
    const float m = float(n * n);
    const float va = saa - sa * sa / m;
    float best = -2.0f, best_prev = -2.0f, best_next = -2.0f, prev = -2.0f;
    int best_k = -1;
    for (int k = 0; k < nc; ++k) {
        float sc = -2.0f;
        if (!(dead & (1u << k))) {
            const float vb = sbb[k] - sb[k] * sb[k] / m;
            sc = (va < 1e-6f || vb < 1e-6f) ? -1.0f : (sab[k] - sa * sb[k] / m) / sqrt(va * vb);
        }
        if (k == best_k + 1) best_next = sc;
        if (sc > best) { best = sc; best_k = k; best_prev = prev; best_next = -2.0f; }
        prev = sc;
    }
    if (best_k <= 0 || best_k + 1 >= nc || best < a.min_zncc || best_prev <= -2.0f || best_next <= -2.0f) return;
    float D = D0 + a.step * float(best_k - a.K);
    if (a.subpixel) D += a.step * subpixel_offset(1.0f - best_prev, 1.0f - best, 1.0f - best_next);
    out[i] = 0.5f * D;
}

// ---- occlusion + spike filters (mirror depth::filter_disparity) ----
struct FilterArgs { uint w; uint h; uint occlusion; float occlusion_tolerance; uint spikes; float spike_threshold; };
constant float kClaimScale = 256.0f;  // claims hold disparities in 1/256 px (integer atomics)

kernel void occlusion_claim(device const float* disp [[buffer(0)]], device atomic_int* claims [[buffer(1)]],
                            constant FilterArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (!a.occlusion || gid.x >= a.w || gid.y >= a.h) return;
    const float d = disp[gid.y * a.w + gid.x];
    if (!valid(d)) return;
    const int xr = int(rint(float(gid.x) - d));
    if (xr < 0 || xr >= int(a.w)) return;
    atomic_fetch_max_explicit(&claims[gid.y * a.w + uint(xr)], int(rint(d * kClaimScale)), memory_order_relaxed);
}

kernel void occlusion_apply(device float* disp [[buffer(0)]], device const int* claims [[buffer(1)]],
                            constant FilterArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (!a.occlusion || gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    const float d = disp[i];
    if (!valid(d)) return;
    const int xr = int(rint(float(gid.x) - d));
    if (xr < 0 || xr >= int(a.w)) return;
    const int c = claims[gid.y * a.w + uint(xr)];
    if (c != kNoMatch && d < float(c) / kClaimScale - a.occlusion_tolerance) disp[i] = kInvalid;
}

kernel void spike_filter(device const float* in [[buffer(0)]], device float* out [[buffer(1)]],
                         constant FilterArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const int x = int(gid.x), y = int(gid.y);
    const uint i = gid.y * a.w + gid.x;
    const float d = in[i];
    out[i] = d;
    if (!a.spikes || !valid(d)) return;
    float buf[9];
    int n = 0;
    for (int v = -1; v <= 1; ++v)
        for (int u = -1; u <= 1; ++u) {
            const int xx = x + u, yy = y + v;
            if (xx < 0 || yy < 0 || xx >= int(a.w) || yy >= int(a.h)) continue;
            const float e = in[yy * int(a.w) + xx];
            if (!valid(e)) continue;
            buf[n++] = e;
        }
    if (n < 3) return;
    for (int i2 = 1; i2 < n; ++i2) {
        const float t = buf[i2];
        int j = i2 - 1;
        while (j >= 0 && buf[j] > t) { buf[j + 1] = buf[j]; --j; }
        buf[j + 1] = t;
    }
    if (abs(d - buf[n / 2]) > a.spike_threshold) out[i] = kInvalid;
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
    atomic_store_explicit(&L[i], valid(disp[i]) ? int(i) : -1, memory_order_relaxed);
    atomic_store_explicit(&sizes[i], 0u, memory_order_relaxed);
}

kernel void ccl_merge(device const float* disp [[buffer(0)]], device atomic_int* L [[buffer(1)]],
                      constant CclArgs& a [[buffer(2)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    const float d = disp[i];
    if (!valid(d)) return;
    if (gid.x + 1 < a.w) {
        const float dr = disp[i + 1];
        if (valid(dr) && abs(dr - d) <= a.max_diff) ccl_unite(L, int(i), int(i + 1));
    }
    if (gid.y + 1 < a.h) {
        const float dd = disp[i + a.w];
        if (valid(dd) && abs(dd - d) <= a.max_diff) ccl_unite(L, int(i), int(i + a.w));
    }
}

kernel void ccl_count(device const float* disp [[buffer(0)]], device atomic_int* L [[buffer(1)]],
                      device atomic_uint* sizes [[buffer(2)]], constant CclArgs& a [[buffer(3)]],
                      uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    if (!valid(disp[i])) return;
    const int root = ccl_find(L, int(i));
    atomic_fetch_add_explicit(&sizes[root], 1u, memory_order_relaxed);
    // Flatten: a root is a valid ancestor for concurrent finds, and ccl_filter then needs one load.
    atomic_store_explicit(&L[i], root, memory_order_relaxed);
}

kernel void ccl_filter(device float* disp [[buffer(0)]], device float* conf [[buffer(1)]], device atomic_int* L [[buffer(2)]],
                       device const atomic_uint* sizes [[buffer(3)]], constant CclArgs& a [[buffer(4)]],
                       uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    if (!valid(disp[i])) { conf[i] = 0.0f; return; }
    const int root = atomic_load_explicit(&L[i], memory_order_relaxed);  // flattened by ccl_count
    if (atomic_load_explicit(&sizes[root], memory_order_relaxed) < a.min_size) {
        disp[i] = kInvalid;
        conf[i] = 0.0f;
    }
}

// ---- disparity -> organised points, normals, weights (mirrors depth::disparity_to_points) ----
struct PointsArgs { uint w; uint h; float f; float cx; float cy; float baseline; float min_depth; float max_depth; float max_jump; float cx_offset;
                   float edge_weight; float weight_reference_depth; };

kernel void disparity_points(device const float* disp [[buffer(0)]], device const float* conf [[buffer(1)]],
                             device float4* pts [[buffer(2)]], constant PointsArgs& a [[buffer(3)]],
                             uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    float4 p = float4(0);
    const float d = disp[i];
    if (valid(d) && d + a.cx_offset > 0) {
        const float z = a.f * a.baseline / (d + a.cx_offset);
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
    float view = a.edge_weight;
    if (p.w > 0 && gid.x > 0 && gid.y > 0 && gid.x + 1 < a.w && gid.y + 1 < a.h) {
        const float4 l = pts[i - 1], r = pts[i + 1], t = pts[i - a.w], b = pts[i + a.w];
        if (l.w > 0 && r.w > 0 && t.w > 0 && b.w > 0 && abs(l.z - r.z) <= a.max_jump && abs(t.z - b.z) <= a.max_jump) {
            float3 nn = cross(r.xyz - l.xyz, b.xyz - t.xyz);
            const float len = length(nn);
            if (len >= 1e-9f) {
                nn /= len;
                if (dot(nn, p.xyz) > 0) nn = -nn;
                n = float4(nn, 0);
                view = max(0.0f, min(-dot(nn, normalize(p.xyz)), -dot(nn, normalize(p.xyz - float3(a.baseline, 0, 0)))));
            }
        }
    }
    if (p.w > 0) {
        const float rz = a.weight_reference_depth / p.z;
        w *= view * (a.weight_reference_depth > 0 ? clamp(rz * rz, 0.1f, 1.0f) : 1.0f);
    }
    nrm[i] = n;
    wts[i] = w;
}
)METAL"
R"METAL(
// ---- marker blob search on the raw IR images (the O(pixels) part of markers::detect_markers) ----
// Connected components of pixels above the threshold (union-find as for the speckle filter), per
// component box / size / peak / second moments by atomics, then the cheap gates of the CPU detector
// and a dark-ring pre-check on the moment ellipse write a short candidate list; the CPU fits
// sub-pixel ellipses to those only.
struct BlobArgs {
    uint w; uint h; uint threshold; uint min_diameter; uint max_diameter; float max_aspect; float min_fill;
    uint border; float max_axis_ratio; float ring_scale; float ring_contrast; float ring_max_bright; uint max_blobs;
};
struct BlobOut { uint x0; uint y0; uint x1; uint y1; uint pixels; uint peak; };

kernel void blob_init(device const uchar* img [[buffer(0)]], device atomic_int* L [[buffer(1)]],
                      device atomic_uint* count [[buffer(2)]], device atomic_uint* bx0 [[buffer(3)]], device atomic_uint* by0 [[buffer(4)]],
                      device atomic_uint* bx1 [[buffer(5)]], device atomic_uint* by1 [[buffer(6)]], device atomic_uint* peak [[buffer(7)]],
                      device atomic_int* mom [[buffer(8)]], constant BlobArgs& a [[buffer(9)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    const bool fg = img[i] > a.threshold;
    atomic_store_explicit(&L[i], fg ? int(i) : -1, memory_order_relaxed);
    if (!fg) return;  // only foreground pixels can be roots, and stats are only read at roots
    atomic_store_explicit(&count[i], 0u, memory_order_relaxed);
    atomic_store_explicit(&bx0[i], 0xFFFFFFFFu, memory_order_relaxed);
    atomic_store_explicit(&by0[i], 0xFFFFFFFFu, memory_order_relaxed);
    atomic_store_explicit(&bx1[i], 0u, memory_order_relaxed);
    atomic_store_explicit(&by1[i], 0u, memory_order_relaxed);
    atomic_store_explicit(&peak[i], 0u, memory_order_relaxed);
    for (uint k = 0; k < 5; ++k) atomic_store_explicit(&mom[5 * i + k], 0, memory_order_relaxed);
}

kernel void blob_merge(device const uchar* img [[buffer(0)]], device atomic_int* L [[buffer(1)]], constant BlobArgs& a [[buffer(2)]],
                       uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    if (img[i] <= a.threshold) return;
    if (gid.x + 1 < a.w && img[i + 1] > a.threshold) ccl_unite(L, int(i), int(i + 1));
    if (gid.y + 1 < a.h && img[i + a.w] > a.threshold) ccl_unite(L, int(i), int(i + a.w));
}

kernel void blob_stats(device const uchar* img [[buffer(0)]], device atomic_int* L [[buffer(1)]],
                       device atomic_uint* count [[buffer(2)]], device atomic_uint* bx0 [[buffer(3)]], device atomic_uint* by0 [[buffer(4)]],
                       device atomic_uint* bx1 [[buffer(5)]], device atomic_uint* by1 [[buffer(6)]], device atomic_uint* peak [[buffer(7)]],
                       device atomic_int* mom [[buffer(8)]], constant BlobArgs& a [[buffer(9)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    if (img[i] <= a.threshold) return;
    const int r = ccl_find(L, int(i));
    // Moments relative to the root pixel, exact in integers: blobs that can pass the size gates span at
    // most max_diameter pixels, so the sums stay far below 2^31 (larger blobs may wrap; they are
    // rejected by their box first). Integer atomics are native everywhere; float adds are not.
    const int dx = int(gid.x) - int(uint(r) % a.w), dy = int(gid.y) - int(uint(r) / a.w);
    uint n = 1, x0 = gid.x, y0 = gid.y, x1 = gid.x, y1 = gid.y, pk = uint(img[i]);
    int m0 = dx, m1 = dy, m2 = dx * dx, m3 = dx * dy, m4 = dy * dy;
    // Inside a large bright region (a white surface under the ring light) every pixel updates the same
    // root: reduce over the SIMD group first, so one lane makes the updates (the same exact sums).
    if (simd_all(r == simd_broadcast_first(r))) {
        n = simd_sum(n);
        x0 = simd_min(x0), y0 = simd_min(y0), x1 = simd_max(x1), y1 = simd_max(y1), pk = simd_max(pk);
        m0 = simd_sum(m0), m1 = simd_sum(m1), m2 = simd_sum(m2), m3 = simd_sum(m3), m4 = simd_sum(m4);
        if (!simd_is_first()) return;
    }
    atomic_fetch_add_explicit(&count[r], n, memory_order_relaxed);
    atomic_fetch_min_explicit(&bx0[r], x0, memory_order_relaxed);
    atomic_fetch_min_explicit(&by0[r], y0, memory_order_relaxed);
    atomic_fetch_max_explicit(&bx1[r], x1, memory_order_relaxed);
    atomic_fetch_max_explicit(&by1[r], y1, memory_order_relaxed);
    atomic_fetch_max_explicit(&peak[r], pk, memory_order_relaxed);
    atomic_fetch_add_explicit(&mom[5 * uint(r) + 0], m0, memory_order_relaxed);
    atomic_fetch_add_explicit(&mom[5 * uint(r) + 1], m1, memory_order_relaxed);
    atomic_fetch_add_explicit(&mom[5 * uint(r) + 2], m2, memory_order_relaxed);
    atomic_fetch_add_explicit(&mom[5 * uint(r) + 3], m3, memory_order_relaxed);
    atomic_fetch_add_explicit(&mom[5 * uint(r) + 4], m4, memory_order_relaxed);
}

inline float sample_img(device const uchar* img, constant BlobArgs& a, float x, float y) {
    const int sx = clamp(int(round(x)), 0, int(a.w) - 1), sy = clamp(int(round(y)), 0, int(a.h) - 1);
    return float(img[uint(sy) * a.w + uint(sx)]);
}

kernel void blob_select(device const uchar* img [[buffer(0)]], device atomic_int* L [[buffer(1)]],
                        device atomic_uint* count [[buffer(2)]], device atomic_uint* bx0 [[buffer(3)]], device atomic_uint* by0 [[buffer(4)]],
                        device atomic_uint* bx1 [[buffer(5)]], device atomic_uint* by1 [[buffer(6)]], device atomic_uint* peak [[buffer(7)]],
                        device atomic_int* mom [[buffer(8)]], device BlobOut* out [[buffer(9)]], device atomic_uint* n_out [[buffer(10)]],
                        constant BlobArgs& a [[buffer(11)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    if (img[i] <= a.threshold || atomic_load_explicit(&L[i], memory_order_relaxed) != int(i)) return;  // roots only
    const uint x0 = atomic_load_explicit(&bx0[i], memory_order_relaxed), y0 = atomic_load_explicit(&by0[i], memory_order_relaxed);
    const uint x1 = atomic_load_explicit(&bx1[i], memory_order_relaxed), y1 = atomic_load_explicit(&by1[i], memory_order_relaxed);
    const uint px = atomic_load_explicit(&count[i], memory_order_relaxed), pk = atomic_load_explicit(&peak[i], memory_order_relaxed);
    const float bw = float(x1 - x0 + 1), bh = float(y1 - y0 + 1);
    // Same gates as the CPU detector (size, aspect, fill, room for the local background ring).
    if (bw < float(a.min_diameter) || bh < float(a.min_diameter) || bw > float(a.max_diameter) || bh > float(a.max_diameter)) return;
    if (max(bw, bh) > a.max_aspect * min(bw, bh)) return;
    if (float(px) < a.min_fill * bw * bh * M_PI_F / 4.0f) return;
    const int m = max(3, int(max(bw, bh)) / 2);
    if (int(x0) - m < int(a.border) || int(y0) - m < int(a.border) || int(x1) + m >= int(a.w - a.border) || int(y1) + m >= int(a.h - a.border))
        return;
    // Ellipse of the blob from its second moments (a filled ellipse has variance (axis/2)^2).
    const float n = float(px);
    const float mx = float(atomic_load_explicit(&mom[5 * i + 0], memory_order_relaxed)) / n;
    const float my = float(atomic_load_explicit(&mom[5 * i + 1], memory_order_relaxed)) / n;
    const float cxx = float(atomic_load_explicit(&mom[5 * i + 2], memory_order_relaxed)) / n - mx * mx;
    const float cxy = float(atomic_load_explicit(&mom[5 * i + 3], memory_order_relaxed)) / n - mx * my;
    const float cyy = float(atomic_load_explicit(&mom[5 * i + 4], memory_order_relaxed)) / n - my * my;
    const float tr = 0.5f * (cxx + cyy), det = sqrt(max(0.0f, 0.25f * (cxx - cyy) * (cxx - cyy) + cxy * cxy));
    const float l1 = tr + det, l2 = max(tr - det, 0.0625f);
    const float ea = 2.0f * sqrt(l1) + 0.5f, eb = 2.0f * sqrt(l2) + 0.5f;  // + half a pixel: the threshold edge sits inside the half-level contour
    if (ea / eb > a.max_axis_ratio * 1.2f) return;
    const float th = 0.5f * atan2(2.0f * cxy, cxx - cyy);
    const float ct = cos(th), st = sin(th);
    const float cx = float(i % a.w) + mx, cy = float(i / a.w) + my;
    // Background: a low sample on the surface outside the sticker (2.2x the disc).
    float bg[16];
    for (int k = 0; k < 16; ++k) {
        const float t = 2.0f * M_PI_F * float(k) / 16.0f;
        const float u = 2.2f * ea * cos(t), v = 2.2f * eb * sin(t);
        bg[k] = sample_img(img, a, cx + ct * u - st * v, cy + st * u + ct * v);
    }
    for (int p = 0; p < 4; ++p)  // partial selection: 4th smallest
        for (int q = p + 1; q < 16; ++q)
            if (bg[q] < bg[p]) { const float tmp = bg[p]; bg[p] = bg[q]; bg[q] = tmp; }
    const float level = bg[3] + a.ring_contrast * (float(pk) - bg[3]);
    // The sticker's dark ring must surround the disc (laser speckle dots sit among bright neighbours).
    int bright = 0;
    for (int k = 0; k < 16; ++k) {
        const float t = 2.0f * M_PI_F * float(k) / 16.0f;
        const float u = a.ring_scale * ea * cos(t), v = a.ring_scale * eb * sin(t);
        bright += sample_img(img, a, cx + ct * u - st * v, cy + st * u + ct * v) > level ? 1 : 0;
    }
    if (float(bright) > a.ring_max_bright * 16.0f) return;
    const uint slot = atomic_fetch_add_explicit(n_out, 1u, memory_order_relaxed);
    if (slot < a.max_blobs) out[slot] = BlobOut{x0, y0, x1, y1, px, pk};
}
)METAL"
