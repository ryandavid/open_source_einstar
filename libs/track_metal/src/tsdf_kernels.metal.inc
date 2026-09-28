R"METAL(
#include <metal_stdlib>
using namespace metal;

// GPU TSDF on a hash of 8^3 bricks. Mirrors libs/track/src/tsdf.cpp (the CPU reference).

constant uint kEmpty = 0xFFFFFFFFu;
constant int kBrick = 8;
constant uint kMaxProbes = 128;

struct VolumeArgs {
    float voxel;
    float trunc;
    float max_weight;
    float min_depth;
    float max_depth;
    uint table_mask;
    uint brick_capacity;
    uint frame;
    float weight_scale;
    uint extend_only;
    float min_weight;   // extraction
    uint since_frame;   // extraction
};

struct CameraArgs {
    float4x4 T_wc;  // camera -> world
    float4x4 T_cw;  // world -> camera
    float fx, fy, cx, cy;
    uint w, h;
};

struct SurfacePoint { packed_float3 p; packed_float3 n; float w; };

// Brick coordinates packed into 32 bits: x, y in 11 bits (+-1024 bricks), z in 10 bits (+-511).
inline bool pack_key(int3 b, thread uint& key) {
    if (b.x < -1024 || b.x > 1023 || b.y < -1024 || b.y > 1023 || b.z < -511 || b.z > 510) return false;
    key = uint(b.x + 1024) | (uint(b.y + 1024) << 11) | (uint(b.z + 512) << 22);
    return true;
}

inline uint hash_key(uint key, uint mask) {
    uint h = key * 2654435761u;
    h ^= h >> 15;
    return h & mask;
}

inline int lookup(device const atomic_uint* keys, device const atomic_int* values, uint mask, int3 b) {
    uint key;
    if (!pack_key(b, key)) return -1;
    uint i = hash_key(key, mask);
    for (uint probe = 0; probe < kMaxProbes; ++probe) {
        const uint k = atomic_load_explicit(&keys[i], memory_order_relaxed);
        if (k == key) return atomic_load_explicit(&values[i], memory_order_relaxed);
        if (k == kEmpty) return -1;
        i = (i + 1) & mask;
    }
    return -1;
}

inline int floor_div8(int a) { return a >= 0 ? a / 8 : -((-a + 7) / 8); }

// Voxel fetch by global voxel coordinate with a one-entry brick cache.
struct VoxelReader {
    device const atomic_uint* keys;
    device const atomic_int* values;
    device const half2* voxels;
    uint mask;
    int3 cached;
    int brick;

    bool get(int3 g, thread float& sdf, thread float& w) {
        const int3 b = int3(floor_div8(g.x), floor_div8(g.y), floor_div8(g.z));
        if (any(b != cached)) {
            cached = b;
            brick = lookup(keys, values, mask, b);
        }
        if (brick < 0) return false;
        const int3 l = g - b * kBrick;
        const half2 v = voxels[uint(brick) * 512u + uint((l.z * kBrick + l.y) * kBrick + l.x)];
        sdf = float(v.x);
        w = float(v.y);
        return w > 0.0f;
    }
};

inline bool trilinear(thread VoxelReader& r, float3 g, thread float& out) {
    const float3 q = g - 0.5f;
    const int3 i0 = int3(floor(q));
    const float3 f = q - float3(i0);
    float acc = 0.0f;
    for (int dz = 0; dz < 2; ++dz)
        for (int dy = 0; dy < 2; ++dy)
            for (int dx = 0; dx < 2; ++dx) {
                float s, w;
                if (!r.get(i0 + int3(dx, dy, dz), s, w)) return false;
                const float wx = dx ? f.x : 1 - f.x, wy = dy ? f.y : 1 - f.y, wz = dz ? f.z : 1 - f.z;
                acc += wx * wy * wz * s;
            }
    out = acc;
    return true;
}

inline bool robust_sample(thread VoxelReader& r, float3 g, thread float& out) {
    if (trilinear(r, g, out)) return true;
    float w;
    return r.get(int3(floor(g)), out, w);
}

// ---- maintenance ----
kernel void init_voxels(device half2* voxels [[buffer(0)]], uint i [[thread_position_in_grid]]) {
    voxels[i] = half2(1.0h, 0.0h);
}

// ---- allocation: bricks covering each observed point's truncation band ----
inline void mark_visible(int brick, device atomic_uint* stamps, device atomic_uint* visible_count, device uint* visible,
                         uint frame, uint capacity) {
    if (atomic_exchange_explicit(&stamps[brick], frame, memory_order_relaxed) != frame) {
        const uint slot = atomic_fetch_add_explicit(visible_count, 1u, memory_order_relaxed);
        if (slot < capacity) visible[slot] = uint(brick);
    }
}

inline void insert_or_find(int3 b, device atomic_uint* keys, device atomic_int* values, device int4* coords,
                           device atomic_uint* brick_count, device atomic_uint* stamps, device atomic_uint* visible_count,
                           device uint* visible, constant VolumeArgs& a) {
    uint key;
    if (!pack_key(b, key)) return;
    uint i = hash_key(key, a.table_mask);
    for (uint probe = 0; probe < kMaxProbes; ++probe) {
        uint k = atomic_load_explicit(&keys[i], memory_order_relaxed);
        // Metal only offers a weak CAS, which may fail spuriously: keep retrying this slot while it
        // still reads empty, otherwise a key could land past an empty slot where lookups never look.
        while (k == kEmpty) {
            uint expected = kEmpty;
            if (atomic_compare_exchange_weak_explicit(&keys[i], &expected, key, memory_order_relaxed, memory_order_relaxed)) {
                const uint idx = atomic_fetch_add_explicit(brick_count, 1u, memory_order_relaxed);
                if (idx >= a.brick_capacity) {
                    atomic_store_explicit(&values[i], -2, memory_order_relaxed);  // pool exhausted
                    return;
                }
                coords[idx] = int4(b, 0);
                atomic_store_explicit(&values[i], int(idx), memory_order_relaxed);
                mark_visible(int(idx), stamps, visible_count, visible, a.frame, a.brick_capacity);
                return;
            }
            k = expected;  // another key claimed the slot, or a spurious failure (still empty -> retry)
        }
        if (k == key) {
            const int v = atomic_load_explicit(&values[i], memory_order_relaxed);
            // v == -1: the inserting thread has not published the index yet; it marks the brick itself.
            if (v >= 0) mark_visible(v, stamps, visible_count, visible, a.frame, a.brick_capacity);
            return;
        }
        i = (i + 1) & a.table_mask;
    }
}

kernel void allocate(device const float4* points [[buffer(0)]], device atomic_uint* keys [[buffer(1)]],
                     device atomic_int* values [[buffer(2)]], device int4* coords [[buffer(3)]],
                     device atomic_uint* brick_count [[buffer(4)]], device atomic_uint* stamps [[buffer(5)]],
                     device atomic_uint* visible_count [[buffer(6)]], device uint* visible [[buffer(7)]],
                     constant VolumeArgs& a [[buffer(8)]], constant CameraArgs& cam [[buffer(9)]],
                     uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= cam.w || gid.y >= cam.h) return;
    const float3 pc = points[gid.y * cam.w + gid.x].xyz;
    if (pc.z < a.min_depth || pc.z > a.max_depth) return;
    const float3 pw = (cam.T_wc * float4(pc, 1)).xyz;
    const float3 origin = cam.T_wc[3].xyz;
    const float3 ray = normalize(pw - origin);
    const float brick_mm = a.voxel * kBrick;
    int3 last = int3(INT_MIN);
    for (int s = -1; s <= 1; ++s) {
        const int3 b = int3(floor((pw + float(s) * a.trunc * ray) / brick_mm));
        if (all(b == last)) continue;
        last = b;
        insert_or_find(b, keys, values, coords, brick_count, stamps, visible_count, visible, a);
    }
}

// ---- integration: one threadgroup (8x8x8) per visible brick ----
kernel void integrate(device const float4* points [[buffer(0)]], device const float* weights [[buffer(1)]],
                      device const int4* coords [[buffer(2)]], device const uint* visible [[buffer(3)]],
                      device half2* voxels [[buffer(4)]], device uint* last_update [[buffer(5)]],
                      constant VolumeArgs& a [[buffer(6)]], constant CameraArgs& cam [[buffer(7)]],
                      uint3 tg3 [[threadgroup_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]]) {
    const uint brick = visible[tg3.x];
    const int3 b = coords[brick].xyz;
    const int3 g = b * kBrick + int3(lid);
    const float3 pw = (float3(g) + 0.5f) * a.voxel;
    const float3 pc = (cam.T_cw * float4(pw, 1)).xyz;
    if (pc.z <= 0) return;
    const int u = int(rint(cam.fx * pc.x / pc.z + cam.cx));
    const int v = int(rint(cam.fy * pc.y / pc.z + cam.cy));
    if (u < 0 || v < 0 || u >= int(cam.w) || v >= int(cam.h)) return;
    const uint pix = uint(v) * cam.w + uint(u);
    const float zm = points[pix].z;
    if (zm <= 0) return;
    const float sdf = zm - pc.z;
    if (sdf < -a.trunc) return;
    const float tsdf = min(1.0f, sdf / a.trunc);
    const float w_new = a.weight_scale * max(weights[pix], 0.05f);
    const uint vi = brick * 512u + (lid.z * 8u + lid.y) * 8u + lid.x;
    const half2 old = voxels[vi];
    const float w_old = float(old.y);
    if (a.extend_only && w_old > 0.0f) return;
    const float s_new = (float(old.x) * w_old + tsdf * w_new) / (w_old + w_new);
    voxels[vi] = half2(half(s_new), half(min(w_old + w_new, a.max_weight)));
    last_update[brick] = a.frame;
}

// ---- raycast: vertex/normal maps in world coordinates ----
struct RaycastArgs { float min_depth; float max_depth; };

kernel void raycast(device const atomic_uint* keys [[buffer(0)]], device const atomic_int* values [[buffer(1)]],
                    device const half2* voxels [[buffer(2)]], device float4* out_points [[buffer(3)]],
                    device float4* out_normals [[buffer(4)]], constant VolumeArgs& a [[buffer(5)]],
                    constant CameraArgs& cam [[buffer(6)]], uint2 gid [[thread_position_in_grid]]) {
    if (gid.x >= cam.w || gid.y >= cam.h) return;
    const uint i = gid.y * cam.w + gid.x;
    out_points[i] = float4(0);
    out_normals[i] = float4(0);
    VoxelReader r{keys, values, voxels, a.table_mask, int3(INT_MIN), -1};
    const float3 dir_c = float3((float(gid.x) - cam.cx) / cam.fx, (float(gid.y) - cam.cy) / cam.fy, 1.0f);
    const float dir_norm = length(dir_c);
    const float3 dir = (cam.T_wc * float4(dir_c / dir_norm, 0)).xyz;
    const float3 origin = cam.T_wc[3].xyz;
    const float inv_voxel = 1.0f / a.voxel;
    const float brick_mm = a.voxel * kBrick;
    float t = a.min_depth * dir_norm;
    const float t_end = a.max_depth * dir_norm + a.trunc;
    float prev_sdf = 1.0f, prev_t = t;
    bool have_prev = false;
    for (int step = 0; step < 4096 && t < t_end; ++step) {
        const float3 p = origin + t * dir;
        const float3 g = p * inv_voxel;
        // Skip unallocated bricks in one jump to the brick exit.
        const int3 b = int3(floor(p / brick_mm));
        if (lookup(keys, values, a.table_mask, b) < 0) {
            const float3 lo = float3(b) * brick_mm, hi = lo + brick_mm;
            const float3 tx = select((lo - p) / dir, (hi - p) / dir, dir > 0);
            float exit = min(min(dir.x != 0 ? tx.x : 1e9f, dir.y != 0 ? tx.y : 1e9f), dir.z != 0 ? tx.z : 1e9f);
            t += max(exit, 0.0f) + 0.01f;
            have_prev = false;
            continue;
        }
        float s;
        if (!robust_sample(r, g, s)) {
            have_prev = false;
            t += a.voxel;
            continue;
        }
        if (have_prev && prev_sdf > 0 && s <= 0) {
            const float t_hit = prev_t + (t - prev_t) * prev_sdf / (prev_sdf - s);
            const float3 hit = origin + t_hit * dir;
            const float3 gh = hit * inv_voxel;
            float sx0, sx1, sy0, sy1, sz0, sz1;
            if (robust_sample(r, gh - float3(1, 0, 0), sx0) && robust_sample(r, gh + float3(1, 0, 0), sx1) &&
                robust_sample(r, gh - float3(0, 1, 0), sy0) && robust_sample(r, gh + float3(0, 1, 0), sy1) &&
                robust_sample(r, gh - float3(0, 0, 1), sz0) && robust_sample(r, gh + float3(0, 0, 1), sz1)) {
                const float3 n = float3(sx1 - sx0, sy1 - sy0, sz1 - sz0);
                const float len = length(n);
                if (len > 1e-6f) {
                    out_points[i] = float4(hit, 1);
                    out_normals[i] = float4(n / len, 0);
                }
            }
            return;
        }
        if (have_prev && prev_sdf < 0 && s < 0) return;
        prev_sdf = s;
        prev_t = t;
        have_prev = true;
        t += max(a.voxel * 0.5f, s * a.trunc * 0.8f);
    }
}

// ---- surface extraction: one threadgroup per brick (optionally only recently updated ones) ----
kernel void extract(device const atomic_uint* keys [[buffer(0)]], device const atomic_int* values [[buffer(1)]],
                    device const half2* voxels [[buffer(2)]], device const int4* coords [[buffer(3)]],
                    device const uint* last_update [[buffer(4)]], device SurfacePoint* out [[buffer(5)]],
                    device atomic_uint* out_count [[buffer(6)]], constant VolumeArgs& a [[buffer(7)]],
                    constant uint& out_capacity [[buffer(8)]], device const uint* brick_list [[buffer(9)]],
                    constant uint& use_list [[buffer(10)]],
                    uint3 tg3 [[threadgroup_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]]) {
    const uint brick = use_list ? brick_list[tg3.x] : tg3.x;
    if (last_update[brick] <= a.since_frame) return;
    const uint vi = brick * 512u + (lid.z * 8u + lid.y) * 8u + lid.x;
    const half2 v = voxels[vi];
    const float sdf = float(v.x), w = float(v.y);
    if (w < a.min_weight || abs(sdf) > 0.5f * a.voxel / a.trunc) return;
    const int3 g = coords[brick].xyz * kBrick + int3(lid);
    VoxelReader r{keys, values, voxels, a.table_mask, int3(INT_MIN), -1};
    float sx0, sx1, sy0, sy1, sz0, sz1, ww;
    if (!r.get(g - int3(1, 0, 0), sx0, ww) || !r.get(g + int3(1, 0, 0), sx1, ww) || !r.get(g - int3(0, 1, 0), sy0, ww) ||
        !r.get(g + int3(0, 1, 0), sy1, ww) || !r.get(g - int3(0, 0, 1), sz0, ww) || !r.get(g + int3(0, 0, 1), sz1, ww))
        return;
    float3 n = float3(sx1 - sx0, sy1 - sy0, sz1 - sz0);
    const float len = length(n);
    if (len < 1e-6f) return;
    n /= len;
    const float3 center = (float3(g) + 0.5f) * a.voxel;
    const uint slot = atomic_fetch_add_explicit(out_count, 1u, memory_order_relaxed);
    if (slot >= out_capacity) return;
    SurfacePoint sp;
    sp.p = packed_float3(center - n * (sdf * a.trunc));
    sp.n = packed_float3(n);
    sp.w = w;
    out[slot] = sp;
}

// ---- surface extraction straight into render vertices (layout of render::PointVertex) ----
struct RenderVertex { packed_float3 p; packed_float3 n; uchar4 color; };

kernel void extract_render(device const atomic_uint* keys [[buffer(0)]], device const atomic_int* values [[buffer(1)]],
                           device const half2* voxels [[buffer(2)]], device const int4* coords [[buffer(3)]],
                           device RenderVertex* out [[buffer(4)]], device atomic_uint* out_count [[buffer(5)]],
                           constant VolumeArgs& a [[buffer(6)]], constant uint& out_capacity [[buffer(7)]],
                           uint3 tg3 [[threadgroup_position_in_grid]], uint3 lid [[thread_position_in_threadgroup]]) {
    const uint brick = tg3.x;
    const uint vi = brick * 512u + (lid.z * 8u + lid.y) * 8u + lid.x;
    const half2 v = voxels[vi];
    const float sdf = float(v.x), w = float(v.y);
    if (w < a.min_weight || abs(sdf) > 0.5f * a.voxel / a.trunc) return;
    const int3 g = coords[brick].xyz * kBrick + int3(lid);
    VoxelReader r{keys, values, voxels, a.table_mask, int3(INT_MIN), -1};
    float sx0, sx1, sy0, sy1, sz0, sz1, ww;
    if (!r.get(g - int3(1, 0, 0), sx0, ww) || !r.get(g + int3(1, 0, 0), sx1, ww) || !r.get(g - int3(0, 1, 0), sy0, ww) ||
        !r.get(g + int3(0, 1, 0), sy1, ww) || !r.get(g - int3(0, 0, 1), sz0, ww) || !r.get(g + int3(0, 0, 1), sz1, ww))
        return;
    float3 n = float3(sx1 - sx0, sy1 - sy0, sz1 - sz0);
    const float len = length(n);
    if (len < 1e-6f) return;
    n /= len;
    const uint slot = atomic_fetch_add_explicit(out_count, 1u, memory_order_relaxed);
    if (slot >= out_capacity) return;
    // Confidence shading (same map as the CPU path): well-observed surface lighter, thin coverage bluish.
    const float c = clamp(w / 8.0f, 0.0f, 1.0f);
    const float grey = 150.0f + 70.0f * c;
    RenderVertex rv;
    rv.p = packed_float3((float3(g) + 0.5f) * a.voxel - n * (sdf * a.trunc));
    rv.n = packed_float3(n);
    rv.color = uchar4(uchar(grey - 40.0f * (1 - c)), uchar(grey - 20.0f * (1 - c)), uchar(grey), 255);
    out[slot] = rv;
}
)METAL"
