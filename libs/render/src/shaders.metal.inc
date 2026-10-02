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

// Surface seen from behind (the inside of the object, points and mesh alike): normals point out of
// the surface towards where it was seen from, so a negative facing marks the back.
constant float3 kBackColor = float3(0.55, 0.42, 0.40);

// Camera-facing disc splats, sized in world units with a pixel floor, headlight shading; points seen
// from behind take kBackColor.
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
    if (length(float3(p.normal)) > 0.1) {
        float3 l = normalize(-view_pos.xyz);  // towards the eye (the headlight)
        float facing = dot(n_view, l);
        if (facing < 0.0) base.rgb = kBackColor;
        if (u.lighting > 0.5) shade = 0.25 + 0.75 * abs(facing);
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
    float2 c = kCorners[vid];
    float r = m.radius * 1.6;  // include the rim
    SplatOut o;
    if (length(float3(m.normal)) < 0.1) {
        // No normal (map markers): camera-facing disc, nudged towards the viewer.
        float4 vp = u.view * float4(float3(m.pos), 1.0);
        vp.xyz += float3(c * r, 0) + normalize(-vp.xyz) * 0.1;
        o.position = u.proj * vp;
    } else {
        float3 n = normalize(float3(m.normal));
        float3 t = normalize(abs(n.z) < 0.9 ? cross(n, float3(0, 0, 1)) : cross(n, float3(1, 0, 0)));
        float3 b = cross(n, t);
        float3 world = float3(m.pos) + (t * c.x + b * c.y) * r + n * 0.05;
        o.position = u.proj * u.view * float4(world, 1.0);
    }
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

// Triangle mesh (processed result): per-vertex normals, two-sided headlight shading.
struct MeshVertex {
    packed_float3 pos;
    packed_float3 normal;
};

struct MeshOut {
    float4 position [[position]];
    float3 view_pos;
    float3 view_normal;
};

vertex MeshOut mesh_vs(uint vid [[vertex_id]], const device MeshVertex* vs [[buffer(0)]], constant Uniforms& u [[buffer(1)]]) {
    MeshOut o;
    const float4 vp = u.view * float4(float3(vs[vid].pos), 1.0);
    o.position = u.proj * vp;
    o.view_pos = vp.xyz;
    o.view_normal = (u.view * float4(float3(vs[vid].normal), 0.0)).xyz;
    return o;
}

// Solid model in its own frame (the scanner), placed by `model`: per-vertex colour, flat faces, lit like
// the calibration app's board view (a light over the eye, a little from above).
struct SolidOut {
    float4 position [[position]];
    float3 view_pos;
    float3 view_normal;
    float4 color;
};

vertex SolidOut solid_vs(uint vid [[vertex_id]], const device PointVertex* vs [[buffer(0)]], constant Uniforms& u [[buffer(1)]],
                         constant float4x4& model [[buffer(2)]]) {
    const PointVertex v = vs[vid];
    const float4 vp = u.view * (model * float4(float3(v.pos), 1.0));
    SolidOut o;
    o.position = u.proj * vp;
    o.view_pos = vp.xyz;
    o.view_normal = (u.view * (model * float4(float3(v.normal), 0.0))).xyz;
    o.color = float4(v.color) / 255.0;
    return o;
}

fragment float4 solid_fs(SolidOut in [[stage_in]]) {
    const float3 n = normalize(in.view_normal);
    const float light = clamp(0.35 + 0.5 * dot(n, normalize(-in.view_pos)) + 0.25 * n.y, 0.2, 1.0);
    return float4(in.color.rgb * light, 1.0);
}

fragment float4 mesh_fs(MeshOut in [[stage_in]]) {
    const float3 n = normalize(in.view_normal);
    const float3 l = normalize(-in.view_pos);
    const float d = dot(n, l);
    // Front faces light grey, back faces (inside of the surface) tinted so holes are recognisable.
    const float3 base = d >= 0 ? float3(0.82, 0.83, 0.86) : kBackColor;
    const float shade = 0.2 + 0.8 * abs(d);
    const float3 h = normalize(l + l);
    const float spec = pow(max(abs(dot(n, h)), 0.0), 40.0) * 0.25;
    return float4(base * shade + spec, 1.0);
}
)METAL"
