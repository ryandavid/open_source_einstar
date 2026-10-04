#include "einstar/recon/consistency.hpp"

#include <algorithm>
#include <cmath>

namespace einstar::recon {
namespace {

Cloud vertex_cloud(const TriangleMesh& m) {
    Cloud c;
    c.points = m.vertices;
    c.normals = m.normals;
    if (c.normals.size() != c.points.size()) {
        TriangleMesh copy;
        copy.vertices = m.vertices;
        copy.triangles = m.triangles;
        copy.compute_normals();
        c.normals = std::move(copy.normals);
    }
    return c;
}

// Twice the signed area of (a, b, p).
inline float edge(const Eigen::Vector2f& a, const Eigen::Vector2f& b, float px, float py) {
    return (b.x() - a.x()) * (py - a.y()) - (b.y() - a.y()) * (px - a.x());
}

}  // namespace

ConsistencyModel::ConsistencyModel(const TriangleMesh& model) : render_(model), vertices_(vertex_cloud(model)), index_(vertices_) {
    // Rendering costs per triangle (most cover no pixel centre at all): a large model is rendered
    // after a decimation far below the tolerances (a fifth of the triangles on a real scan).
    if (render_.triangles.size() > 200000) {
        SimplifyParams sp;
        sp.max_error_mm = 0.02;
        simplify(render_, sp);
    }
}

ModelView ConsistencyModel::render(const SE3& T_world_camera, const track::Intrinsics& k) const {
    const int w = k.width, h = k.height;
    ModelView view{ImageF32(w, h, 0.0f), Image<Vec3f>(w, h, Vec3f::Zero())};
    const SE3 T_cw = T_world_camera.inverse();
    const Eigen::Matrix3f R = T_cw.linear().cast<float>();
    const Vec3f t = T_cw.translation().cast<float>();
    const auto fx = static_cast<float>(k.fx), fy = static_cast<float>(k.fy), cx = static_cast<float>(k.cx), cy = static_cast<float>(k.cy);
    constexpr float kNear = 1.0f;  // mm; triangles reaching closer to the camera are skipped
    const auto& V = render_.vertices;
    std::vector<Vec3f> pc(V.size());
    std::vector<Eigen::Vector2f> uv(V.size());
    for (std::size_t i = 0; i < V.size(); ++i) {
        pc[i] = R * V[i] + t;
        if (pc[i].z() > kNear) uv[i] = Eigen::Vector2f(fx * pc[i].x() / pc[i].z() + cx, fy * pc[i].y() / pc[i].z() + cy);
    }
    for (const auto& tri : render_.triangles) {
        const Vec3f &a = pc[tri[0]], &b = pc[tri[1]], &c = pc[tri[2]];
        if (a.z() <= kNear || b.z() <= kNear || c.z() <= kNear) continue;
        const Eigen::Vector2f &A = uv[tri[0]], &B = uv[tri[1]], &C = uv[tri[2]];
        // Pixel centres (integer coordinates) inside the triangle's bounding box.
        const int x0 = std::max(0, static_cast<int>(std::ceil(std::min({A.x(), B.x(), C.x()}))));
        const int x1 = std::min(w - 1, static_cast<int>(std::floor(std::max({A.x(), B.x(), C.x()}))));
        if (x0 > x1) continue;
        const int y0 = std::max(0, static_cast<int>(std::ceil(std::min({A.y(), B.y(), C.y()}))));
        const int y1 = std::min(h - 1, static_cast<int>(std::floor(std::max({A.y(), B.y(), C.y()}))));
        if (y0 > y1) continue;
        const float area = edge(A, B, C.x(), C.y());
        if (std::abs(area) < 1e-12f) continue;
        Vec3f n = (b - a).cross(c - a);
        const float len = n.norm();
        if (len <= 0) continue;
        n /= len;
        const float zmin = std::min({a.z(), b.z(), c.z()}), zmax = std::max({a.z(), b.z(), c.z()});
        const float plane = n.dot(a);
        const float s = area > 0 ? 1.0f : -1.0f;
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) {
                const auto px = static_cast<float>(x), py = static_cast<float>(y);
                // Inclusive edges, with slack for rounding: neighbouring triangles leave no gaps (a
                // pixel centre on a shared edge may be drawn twice).
                constexpr float kSlack = -1e-3f;  // px^2
                if (s * edge(B, C, px, py) < kSlack || s * edge(C, A, px, py) < kSlack || s * edge(A, B, px, py) < kSlack) continue;
                // Depth where the pixel's ray meets the triangle's plane.
                const Vec3f r((px - cx) / fx, (py - cy) / fy, 1.0f);
                const float den = n.dot(r);
                const float z = std::abs(den) > 1e-6f ? std::clamp(plane / den, zmin, zmax) : 0.5f * (zmin + zmax);
                float& zb = view.depth(x, y);
                if (zb <= 0 || z < zb) {
                    zb = z;
                    view.normal(x, y) = n;
                }
            }
    }
    return view;
}

ConsistencyStats ConsistencyModel::filter(track::DepthFrame& f, const SE3& T_world_camera, const ConsistencyParams& p) const {
    f.ensure_cpu();
    const auto& k = f.intrinsics;
    const ModelView view = render(T_world_camera, k);
    const Eigen::Matrix3f R = T_world_camera.linear().cast<float>();
    const Vec3f t = T_world_camera.translation().cast<float>();
    const auto cos_normal = p.max_normal_angle_deg > 0 ? static_cast<float>(std::cos(p.max_normal_angle_deg * M_PI / 180.0)) : -2.0f;
    // Is there a model surface facing the camera within `tol` of the (camera-frame) point?
    auto supported = [&](const Vec3f& pc, float tol) {
        const Vec3f pw = R * pc + t;
        for (const auto i : index_.radius(pw, p.support_radius_mm)) {
            const Vec3f& v = vertices_.points[i];
            const Vec3f& n = vertices_.normals[i];
            if (n.dot(v - t) >= 0) continue;  // faces away from this camera
            if (std::abs((pw - v).dot(n)) <= tol) return true;
        }
        return false;
    };
    ConsistencyStats st;
    for (int y = 0; y < f.points.height(); ++y)
        for (int x = 0; x < f.points.width(); ++x) {
            Vec3f& pt = f.points(x, y);
            if (pt.z() <= 0) continue;
            ++st.pixels;
            const float zh = view.depth(x, y);
            if (zh <= 0 || !view.front_facing(x, y, k)) {
                ++st.unseen;
                continue;
            }
            const float tol = p.tolerance(pt.z());
            const Vec3f& n = view.normal(x, y);
            const Vec3f r(static_cast<float>((x - k.cx) / k.fx), static_cast<float>((y - k.cy) / k.fy), 1.0f);
            const float dz = pt.z() - zh;  // > 0: behind the surface
            const float d = dz * n.dot(r);  // along the surface normal, > 0 in front
            std::size_t* verdict = nullptr;
            if (std::abs(d) <= tol && std::abs(dz) * r.norm() <= p.max_ray_tolerances * tol) {
                const Vec3f& pn = f.normals(x, y);
                if (cos_normal > -1 && pn.squaredNorm() > 0 && pn.dot(n) < cos_normal) verdict = &st.normal;
            } else if (!supported(pt, tol)) {
                verdict = dz < 0 ? &st.in_front : &st.behind;
            }
            if (!verdict) continue;
            ++*verdict;
            pt = Vec3f::Zero();
            f.normals(x, y) = Vec3f::Zero();
            if (!f.weights.empty()) f.weights(x, y) = 0.0f;
        }
    return st;
}

}  // namespace einstar::recon
