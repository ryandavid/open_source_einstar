R"METAL(
#include <metal_stdlib>
using namespace metal;

struct Uniforms {
    float4x4 view;
    float4x4 proj;
    float2 viewport;
    float point_size;
    float min_point_px;
    float lighting;
};

struct PointVertex {
    packed_float3 pos;
    packed_float3 normal;
    uchar4 color;
};

struct MarkerInstance {
    packed_float3 pos;
    packed_float3 normal;
    float radius;
    uchar4 color;
};

struct LineVertex {
    packed_float3 pos;
    uchar4 color;
};

struct SplatOut {
    float4 position [[position]];
    float2 corner;
    float4 color;
};

constant float2 kCorners[4] = { float2(-1, -1), float2(1, -1), float2(-1, 1), float2(1, 1) };

// Camera-facing disc splats, sized in world units with a pixel floor, headlight shading.
vertex SplatOut splat_vs(uint vid [[vertex_id]], uint iid [[instance_id]],
                         const device PointVertex* pts [[buffer(0)]],
                         constant Uniforms& u [[buffer(1)]]) {
    PointVertex p = pts[iid];
    float4 view_pos = u.view * float4(float3(p.pos), 1.0);
    float3 n_view = normalize((u.view * float4(float3(p.normal), 0.0)).xyz);
    float2 c = kCorners[vid];
    float4 clip = u.proj * view_pos;
    // world-size radius projected to pixels, clamped to a minimum
    float px_per_mm = u.proj[1][1] * u.viewport.y * 0.5 / max(-view_pos.z, 1e-3);
    float radius_px = max(0.5 * u.point_size * px_per_mm, u.min_point_px);
    clip.xy += c * radius_px * 2.0 / u.viewport * clip.w;

    float4 base = float4(p.color) / 255.0;
    float shade = 1.0;
    if (u.lighting > 0.5 && length(float3(p.normal)) > 0.1) {
        float3 l = normalize(-view_pos.xyz);  // headlight
        shade = 0.25 + 0.75 * abs(dot(n_view, l));
    }
    SplatOut o;
    o.position = clip;
    o.corner = c;
    o.color = float4(base.rgb * shade, base.a);
    return o;
}

fragment float4 splat_fs(SplatOut in [[stage_in]]) {
    if (dot(in.corner, in.corner) > 1.0) discard_fragment();
    return in.color;
}

// Oriented marker discs: white centre, dark ring, state-coloured rim.
vertex SplatOut marker_vs(uint vid [[vertex_id]], uint iid [[instance_id]],
                          const device MarkerInstance* ms [[buffer(0)]],
                          constant Uniforms& u [[buffer(1)]]) {
    MarkerInstance m = ms[iid];
    float3 n = normalize(float3(m.normal));
    float3 t = normalize(abs(n.z) < 0.9 ? cross(n, float3(0, 0, 1)) : cross(n, float3(1, 0, 0)));
    float3 b = cross(n, t);
    float2 c = kCorners[vid];
    float r = m.radius * 1.6;  // include the rim
    float3 world = float3(m.pos) + (t * c.x + b * c.y) * r + n * 0.05;
    SplatOut o;
    o.position = u.proj * u.view * float4(world, 1.0);
    o.corner = c;
    o.color = float4(m.color) / 255.0;
    return o;
}

fragment float4 marker_fs(SplatOut in [[stage_in]]) {
    float d = length(in.corner);  // 1.0 == rim edge; marker radius at 1/1.6
    if (d > 1.0) discard_fragment();
    const float inner = 1.0 / 1.6;
    if (d < inner * 0.55) return float4(1, 1, 1, 1);
    if (d < inner) return float4(0.05, 0.05, 0.05, 1);
    return in.color;
}

struct LineOut {
    float4 position [[position]];
    float4 color;
};

vertex LineOut line_vs(uint vid [[vertex_id]], const device LineVertex* vs [[buffer(0)]],
                       constant Uniforms& u [[buffer(1)]]) {
    LineOut o;
    o.position = u.proj * u.view * float4(float3(vs[vid].pos), 1.0);
    o.color = float4(vs[vid].color) / 255.0;
    return o;
}

fragment float4 line_fs(LineOut in [[stage_in]]) { return in.color; }
)METAL"
