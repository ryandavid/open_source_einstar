R"METAL(
#include <metal_stdlib>
using namespace metal;

// GPU point-to-plane ICP mirroring libs/track/src/icp.cpp. All Gauss-Newton iterations run in one
// command buffer: an accumulate pass (per-threadgroup partial normal equations) followed by a
// single-threadgroup solve pass that updates the pose in device memory.

struct IcpState {
    float T[16];       // current camera -> world pose (column-major)
    float T_init[16];  // prediction (prior target)
    uint failed;
    uint level_done;
    uint stats_written;
    int correspondences;
    int candidates;
    int degenerate_dirs;
    float rms;
    float inlier_ratio;
    float coverage;
    float eig_ratio;
    float pad[2];
    float basis[36];   // column-major 6x6, first degenerate_dirs columns valid
    float hessian[36]; // unit-scaled Hessian of the final solve (row-major), for diagnostics
};

struct AccArgs {
    uint W, H, step;
    float gate, huber, cos_max;
    float mfx, mfy, mcx, mcy;
    uint mw, mh;
    float M_cw[16];    // world -> model camera
    float c[4];        // centring point (world)
};

struct SolveArgs {
    uint num_partials;
    uint final_level;        // this level is the finest (stats are recorded here)
    uint last_iter;          // last iteration of this level
    float lambda_t;          // prior weights in unit-scaled coordinates
    float lambda_r;
    float degenerate_ratio;
    float c[4];
};

constant uint kP = 32;   // floats per partial record: 21 A + 6 b + sq + n + considered + on_model + pad

inline float4x4 load4(device const float* m) {
    return float4x4(float4(m[0], m[1], m[2], m[3]), float4(m[4], m[5], m[6], m[7]), float4(m[8], m[9], m[10], m[11]),
                    float4(m[12], m[13], m[14], m[15]));
}
inline float4x4 load4c(constant float* m) {
    return float4x4(float4(m[0], m[1], m[2], m[3]), float4(m[4], m[5], m[6], m[7]), float4(m[8], m[9], m[10], m[11]),
                    float4(m[12], m[13], m[14], m[15]));
}
inline void store4(device float* m, float4x4 t) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) m[c * 4 + r] = t[c][r];
}

kernel void icp_level_begin(device IcpState* st [[buffer(0)]]) { st->level_done = 0; }

kernel void icp_accumulate(device const float4* src_pts [[buffer(0)]], device const float4* src_nrm [[buffer(1)]],
                           device const float* src_w [[buffer(2)]], device const float4* mdl_pts [[buffer(3)]],
                           device const float4* mdl_nrm [[buffer(4)]], device IcpState* st [[buffer(5)]],
                           device float* partials [[buffer(6)]], constant AccArgs& a [[buffer(7)]],
                           uint2 gid [[thread_position_in_grid]], uint2 tgid [[threadgroup_position_in_grid]],
                           uint2 tgs [[threadgroups_per_grid]], uint lidx [[thread_index_in_threadgroup]],
                           uint lane [[thread_index_in_simdgroup]], uint simd_id [[simdgroup_index_in_threadgroup]],
                           uint nsimd [[simdgroups_per_threadgroup]]) {
    if (st->failed || st->level_done) return;  // uniform across the grid
    float acc[31];
    for (int j = 0; j < 31; ++j) acc[j] = 0.0f;
    const float4x4 T = load4(st->T);
    const float4x4 Mcw = load4c(a.M_cw);
    const float3 c = float3(a.c[0], a.c[1], a.c[2]);
    for (uint rep = 0; rep < 4; ++rep) {
    const uint x = (gid.x * 4 + rep) * a.step, y = gid.y * a.step;
    if (x < a.W && y < a.H) {
        const uint i = y * a.W + x;
        const float4 pc = src_pts[i];
        if (pc.z > 0) {
            acc[29] += 1.0f;  // considered
            const float3 pw = (T * float4(pc.xyz, 1)).xyz;
            const float3 pm = (Mcw * float4(pw, 1)).xyz;
            if (pm.z > 0) {
                const int u = int(rint(a.mfx * pm.x / pm.z + a.mcx));
                const int v = int(rint(a.mfy * pm.y / pm.z + a.mcy));
                if (u >= 0 && v >= 0 && u < int(a.mw) && v < int(a.mh)) {
                    const uint mi = uint(v) * a.mw + uint(u);
                    const float4 q4 = mdl_pts[mi];
                    if (q4.w > 0) {
                        acc[30] += 1.0f;  // on model
                        const float3 q = q4.xyz;
                        const float3 nq = mdl_nrm[mi].xyz;
                        const float r = dot(nq, pw - q);
                        bool ok = !(abs(r) > a.gate || length(pw - q) > 2 * a.gate);
                        const float3 ns = src_nrm[i].xyz;
                        if (ok && dot(ns, ns) > 0) {
                            const float3 nsw = (T * float4(ns, 0)).xyz;
                            if (dot(nsw, nq) < a.cos_max) ok = false;
                        }
                        if (ok) {
                            const float w_h = abs(r) <= a.huber ? 1.0f : a.huber / abs(r);
                            const float w = w_h * src_w[i];
                            float J[6];
                            J[0] = nq.x; J[1] = nq.y; J[2] = nq.z;
                            const float3 jr = cross(pw - c, nq);  // centred rotation Jacobian (better float precision)
                            J[3] = jr.x; J[4] = jr.y; J[5] = jr.z;
                            int k = 0;
                            for (int r0 = 0; r0 < 6; ++r0)
                                for (int c0 = r0; c0 < 6; ++c0) acc[k++] += w * J[r0] * J[c0];
                            for (int j = 0; j < 6; ++j) acc[21 + j] -= w * J[j] * r;
                            acc[27] += w * r * r;
                            acc[28] += 1.0f;
                        }
                    }
                }
            }
        }
    }
    }
    threadgroup float tg_sum[32 * 31];
    for (int j = 0; j < 31; ++j) {
        const float s = simd_sum(acc[j]);
        if (lane == 0) tg_sum[simd_id * 31 + uint(j)] = s;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    if (lidx < 31) {
        float s = 0;
        for (uint g = 0; g < nsimd; ++g) s += tg_sum[g * 31 + lidx];
        partials[(tgid.y * tgs.x + tgid.x) * kP + lidx] = s;
    }
}

// ---- small dense linear algebra for the solve thread ----
typedef float Mat6[6][6];

inline void jacobi6(thread Mat6& A, thread Mat6& V, thread float* ev) {
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) V[i][j] = i == j ? 1.0f : 0.0f;
    for (int sweep = 0; sweep < 20; ++sweep) {
        float off = 0;
        for (int p = 0; p < 6; ++p)
            for (int q = p + 1; q < 6; ++q) off += A[p][q] * A[p][q];
        if (off < 1e-20f) break;
        for (int p = 0; p < 6; ++p)
            for (int q = p + 1; q < 6; ++q) {
                if (abs(A[p][q]) < 1e-30f) continue;
                const float theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
                // Note: sign(0) is 0 in Metal, which would stall pairs with equal diagonals.
                const float t = theta >= 0 ? 1.0f / (theta + sqrt(theta * theta + 1)) : -1.0f / (-theta + sqrt(theta * theta + 1));
                const float c = 1 / sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 6; ++k) {
                    const float akp = A[k][p], akq = A[k][q];
                    A[k][p] = c * akp - s * akq;
                    A[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 6; ++k) {
                    const float apk = A[p][k], aqk = A[q][k];
                    A[p][k] = c * apk - s * aqk;
                    A[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 6; ++k) {
                    const float vkp = V[k][p], vkq = V[k][q];
                    V[k][p] = c * vkp - s * vkq;
                    V[k][q] = s * vkp + c * vkq;
                }
            }
    }
    for (int i = 0; i < 6; ++i) ev[i] = A[i][i];
    // sort ascending (eigenvectors are columns of V)
    for (int i = 0; i < 6; ++i)
        for (int j = i + 1; j < 6; ++j)
            if (ev[j] < ev[i]) {
                const float t = ev[i]; ev[i] = ev[j]; ev[j] = t;
                for (int k = 0; k < 6; ++k) { const float tv = V[k][i]; V[k][i] = V[k][j]; V[k][j] = tv; }
            }
}

inline bool cholesky_solve6(thread Mat6& A, thread float* b, thread float* x) {
    float L[6][6];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j <= i; ++j) {
            float s = A[i][j];
            for (int k = 0; k < j; ++k) s -= L[i][k] * L[j][k];
            if (i == j) {
                if (s <= 0) return false;
                L[i][i] = sqrt(s);
            } else {
                L[i][j] = s / L[j][j];
            }
        }
    float y[6];
    for (int i = 0; i < 6; ++i) {
        float s = b[i];
        for (int k = 0; k < i; ++k) s -= L[i][k] * y[k];
        y[i] = s / L[i][i];
    }
    for (int i = 5; i >= 0; --i) {
        float s = y[i];
        for (int k = i + 1; k < 6; ++k) s -= L[k][i] * x[k];
        x[i] = s / L[i][i];
    }
    return true;
}

inline float3x3 hat(float3 w) { return float3x3(float3(0, w.z, -w.y), float3(-w.z, 0, w.x), float3(w.y, -w.x, 0)); }

inline float4x4 se3_exp(thread const float* xi) {
    const float3 v = float3(xi[0], xi[1], xi[2]), w = float3(xi[3], xi[4], xi[5]);
    const float th = length(w);
    const float3x3 W = hat(w);
    const float3x3 I = float3x3(1.0f);
    float3x3 R, V;
    if (th < 1e-7f) {
        R = I + W;
        V = I + 0.5f * W;
    } else {
        const float s = sin(th), c = cos(th);
        R = I + (s / th) * W + ((1 - c) / (th * th)) * (W * W);
        V = I + ((1 - c) / (th * th)) * W + ((th - s) / (th * th * th)) * (W * W);
    }
    const float3 t = V * v;
    return float4x4(float4(R[0], 0), float4(R[1], 0), float4(R[2], 0), float4(t, 1));
}

inline void se3_log(float4x4 T, thread float* xi) {
    const float3x3 R = float3x3(T[0].xyz, T[1].xyz, T[2].xyz);
    const float3 skew = float3(R[1][2] - R[2][1], R[2][0] - R[0][2], R[0][1] - R[1][0]);  // 2 sin(th) * axis
    // atan2 keeps small angles accurate in float (acos of a trace near 1 does not).
    const float s = 0.5f * length(skew);
    const float c = 0.5f * (R[0][0] + R[1][1] + R[2][2] - 1);
    const float th = atan2(s, c);
    const float3 w = s > 1e-7f ? (th / s) * 0.5f * skew : 0.5f * skew;
    const float3x3 W = hat(w);
    const float3x3 I = float3x3(1.0f);
    float3x3 Vinv;
    if (th < 1e-6f) Vinv = I - 0.5f * W;
    else {
        const float half_th = 0.5f * th;
        Vinv = I - 0.5f * W + (1 - half_th * cos(half_th) / sin(half_th)) / (th * th) * (W * W);
    }
    const float3 v = Vinv * T[3].xyz;
    xi[0] = v.x; xi[1] = v.y; xi[2] = v.z; xi[3] = w.x; xi[4] = w.y; xi[5] = w.z;
}

inline float4x4 inverse_rigid(float4x4 T) {
    const float3x3 R = float3x3(T[0].xyz, T[1].xyz, T[2].xyz);
    const float3x3 Rt = transpose(R);
    const float3 t = -(Rt * T[3].xyz);
    return float4x4(float4(Rt[0], 0), float4(Rt[1], 0), float4(Rt[2], 0), float4(t, 1));
}

kernel void icp_solve(device IcpState* st [[buffer(0)]], device const float* partials [[buffer(1)]],
                      constant SolveArgs& a [[buffer(2)]], uint lidx [[thread_index_in_threadgroup]],
                      uint tgsize [[threads_per_threadgroup]], uint lane [[thread_index_in_simdgroup]],
                      uint simd_id [[simdgroup_index_in_threadgroup]], uint nsimd [[simdgroups_per_threadgroup]]) {
    if (st->failed || st->level_done) return;
    threadgroup float totals[31];
    threadgroup float tg_sum[32 * 31];
    float acc[31];
    for (int j = 0; j < 31; ++j) acc[j] = 0.0f;
    for (uint i = lidx; i < a.num_partials; i += tgsize)
        for (int j = 0; j < 31; ++j) acc[j] += partials[i * kP + uint(j)];
    for (int j = 0; j < 31; ++j) {
        const float v = simd_sum(acc[j]);
        if (lane == 0) tg_sum[simd_id * 31 + uint(j)] = v;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    if (lidx < 31) {
        float v = 0;
        for (uint g = 0; g < nsimd; ++g) v += tg_sum[g * 31 + lidx];
        totals[lidx] = v;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    if (lidx != 0) return;

    const float n = totals[28];
    if (n < 50) {
        st->failed = 1;
        st->correspondences = int(n);
        return;
    }
    // Normal equations about the centroid c (same convention as the CPU solver).
    Mat6 Ao;
    float bo[6];
    int k = 0;
    for (int r = 0; r < 6; ++r)
        for (int c0 = r; c0 < 6; ++c0) { Ao[r][c0] = totals[k]; Ao[c0][r] = totals[k]; ++k; }
    for (int j = 0; j < 6; ++j) bo[j] = totals[21 + j];
    const float3 c = float3(a.c[0], a.c[1], a.c[2]);
    // Unit-scaled system (rotations as displacement at 100 mm).
    const float sc[6] = {1, 1, 1, 0.01f, 0.01f, 0.01f};
    Mat6 As, Ar, E, V;
    float bs[6], rhs[6], xs[6], ev[6], prior_s[6];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j) As[i][j] = sc[i] * Ao[i][j] * sc[j];
    for (int i = 0; i < 6; ++i) bs[i] = sc[i] * bo[i];
    const float4x4 T = load4(st->T);
    const float4x4 Tinit = load4(st->T_init);
    float dxp[6];
    se3_log(Tinit * inverse_rigid(T), dxp);
    {  // origin twist -> twist about c: v_c = v_o - c x w
        const float3 vc = float3(dxp[0], dxp[1], dxp[2]) - cross(c, float3(dxp[3], dxp[4], dxp[5]));
        dxp[0] = vc.x; dxp[1] = vc.y; dxp[2] = vc.z;
    }
    for (int i = 0; i < 6; ++i) prior_s[i] = dxp[i] / sc[i];
    const float lam[6] = {a.lambda_t, a.lambda_t, a.lambda_t, a.lambda_r, a.lambda_r, a.lambda_r};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) { Ar[i][j] = As[i][j]; E[i][j] = As[i][j]; }
        Ar[i][i] += lam[i];
        rhs[i] = bs[i] + lam[i] * prior_s[i];
    }
    if (!cholesky_solve6(Ar, rhs, xs)) {
        st->failed = 1;
        return;
    }
    jacobi6(E, V, ev);
    int degenerate = 0;
    float basis[36];
    for (int i = 0; i < 36; ++i) basis[i] = 0;
    if (ev[5] > 0) {
        for (int d = 0; d < 6; ++d) {
            if (ev[d] >= a.degenerate_ratio * ev[5]) continue;
            float vp = 0, vx = 0;
            for (int i = 0; i < 6; ++i) { vp += V[i][d] * prior_s[i]; vx += V[i][d] * xs[i]; }
            for (int i = 0; i < 6; ++i) xs[i] += V[i][d] * (vp - vx);
            for (int i = 0; i < 6; ++i) basis[degenerate * 6 + i] = V[i][d];
            ++degenerate;
        }
    }
    float dx[6];
    for (int i = 0; i < 6; ++i) dx[i] = sc[i] * xs[i];
    float dxo[6] = {dx[0], dx[1], dx[2], dx[3], dx[4], dx[5]};
    {  // twist about c -> origin twist: v_o = v_c + c x w
        const float3 vo = float3(dx[0], dx[1], dx[2]) + cross(c, float3(dx[3], dx[4], dx[5]));
        dxo[0] = vo.x; dxo[1] = vo.y; dxo[2] = vo.z;
    }
    store4(st->T, se3_exp(dxo) * T);

    const bool converged = length(float3(dx[0], dx[1], dx[2])) < 1e-4f && length(float3(dx[3], dx[4], dx[5])) < 1e-6f;
    if (a.final_level && (a.last_iter || converged)) {
        st->stats_written = 1;
        st->correspondences = int(n);
        st->candidates = int(totals[29]);
        st->rms = sqrt(totals[27] / max(1.0f, n));
        st->inlier_ratio = totals[30] > 0 ? n / totals[30] : 0.0f;
        st->coverage = totals[29] > 0 ? totals[30] / totals[29] : 0.0f;
        st->eig_ratio = ev[5] > 0 ? ev[0] / ev[5] : 0.0f;
        st->degenerate_dirs = degenerate;
        for (int i = 0; i < 36; ++i) st->basis[i] = basis[i];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) st->hessian[i * 6 + j] = As[i][j];
    }
    if (converged) st->level_done = 1;
}

// ---- normal-space balancing weights (mirrors track::normal_balance_weights) ----
struct BalanceArgs { uint w; uint h; float alpha; uint enabled; };

inline int normal_bin(float3 n) {
    const float theta = acos(clamp(n.z, -1.0f, 1.0f));
    const float phi = atan2(n.y, n.x) + M_PI_F;
    const int bt = min(7, int(theta / M_PI_F * 8.0f));
    const int bp = min(15, int(phi / (2.0f * M_PI_F) * 16.0f));
    return bt * 16 + bp;
}

kernel void balance_hist(device const float4* pts [[buffer(0)]], device const float4* nrm [[buffer(1)]],
                         device atomic_uint* hist [[buffer(2)]], constant BalanceArgs& a [[buffer(3)]],
                         uint2 gid [[thread_position_in_grid]], uint lidx [[thread_index_in_threadgroup]]) {
    threadgroup atomic_uint local[128];
    if (lidx < 128) atomic_store_explicit(&local[lidx], 0u, memory_order_relaxed);
    threadgroup_barrier(mem_flags::mem_threadgroup);
    if (gid.x < a.w && gid.y < a.h) {
        const uint i = gid.y * a.w + gid.x;
        const float3 n = nrm[i].xyz;
        if (pts[i].z > 0 && dot(n, n) > 0) atomic_fetch_add_explicit(&local[normal_bin(n)], 1u, memory_order_relaxed);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    if (lidx < 128) {
        const uint c = atomic_load_explicit(&local[lidx], memory_order_relaxed);
        if (c) atomic_fetch_add_explicit(&hist[lidx], c, memory_order_relaxed);
    }
}

kernel void balance_bins(device const uint* hist [[buffer(0)]], device float* w_bin [[buffer(1)]],
                         constant BalanceArgs& a [[buffer(2)]], uint tid [[thread_position_in_grid]]) {
    if (tid != 0) return;
    uint total = 0, occupied = 0;
    for (int b = 0; b < 128; ++b) { total += hist[b]; occupied += hist[b] > 0; }
    const float mean = occupied ? float(total) / float(occupied) : 1.0f;
    for (int b = 0; b < 128; ++b) w_bin[b] = hist[b] ? min(pow(mean / float(hist[b]), a.alpha), 20.0f) : 0.0f;
}

kernel void balance_apply(device const float4* nrm [[buffer(0)]], device const float* wts [[buffer(1)]],
                          device const float* w_bin [[buffer(2)]], device float* src_w [[buffer(3)]],
                          constant BalanceArgs& a [[buffer(4)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= a.w || gid.y >= a.h) return;
    const uint i = gid.y * a.w + gid.x;
    float w = max(0.1f, wts[i]);
    const float3 n = nrm[i].xyz;
    if (a.enabled && dot(n, n) > 0) w *= w_bin[normal_bin(n)];
    src_w[i] = w;
}
)METAL"
