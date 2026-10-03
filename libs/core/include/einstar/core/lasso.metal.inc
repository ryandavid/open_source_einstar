R"METAL(
// LassoSelection::contains (einstar/core/lasso.hpp) on the GPU: the same rule on the same masks. Prepended
// to the shader sources that use it (so self-contained).
#include <metal_stdlib>
using namespace metal;

struct LassoStroke {
    float4x4 view_proj;
    float2 viewport;
    int2 origin;
    int2 size;
    uint offset;
    uint subtract;
};

inline bool lasso_contains(float3 p, const device LassoStroke* strokes, uint count, const device uchar* masks) {
    bool selected = false;
    for (uint i = 0; i < count; ++i) {
        const LassoStroke s = strokes[i];
        const float4 clip = s.view_proj * float4(p, 1.0);
        bool inside = false;
        if (clip.w > 1e-6f) {
            const float px = (clip.x / clip.w * 0.5f + 0.5f) * s.viewport.x - float(s.origin.x);
            const float py = (0.5f - clip.y / clip.w * 0.5f) * s.viewport.y - float(s.origin.y);
            if (px >= 0 && py >= 0 && px < float(s.size.x) && py < float(s.size.y))
                inside = masks[s.offset + uint(py) * uint(s.size.x) + uint(px)] != 0;
        }
        selected = s.subtract != 0 ? (selected && !inside) : (selected || inside);
    }
    return selected;
}
)METAL"
