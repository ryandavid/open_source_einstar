#include "einstar/track/tsdf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

namespace einstar::track {
namespace {

inline int floor_div(int a, int b) { return a >= 0 ? a / b : -((-a + b - 1) / b); }

inline int voxel_index(int x, int y, int z) { return (z * kBrickSize + y) * kBrickSize + x; }

// Global voxel coordinate lookup with a one-entry brick cache.
struct VoxelReader {
    const std::unordered_map<BrickCoord, std::unique_ptr<Brick>, BrickCoordHash>& map;
    BrickCoord cached{std::numeric_limits<int>::min(), 0, 0};
    const Brick* brick = nullptr;

    bool get(int gx, int gy, int gz, float& sdf, float& w) {
        const BrickCoord c{floor_div(gx, kBrickSize), floor_div(gy, kBrickSize), floor_div(gz, kBrickSize)};
        if (!(c == cached)) {
            cached = c;
            auto it = map.find(c);
            brick = it == map.end() ? nullptr : it->second.get();
        }
        if (!brick) return false;
        const int i = voxel_index(gx - c.x * kBrickSize, gy - c.y * kBrickSize, gz - c.z * kBrickSize);
        w = brick->weight[static_cast<std::size_t>(i)];
        sdf = brick->sdf[static_cast<std::size_t>(i)];
        return w > 0.0f;
    }
};

bool trilinear_sample(VoxelReader& r, const Vec3f& g, float& out) {
    // g is in voxel units where voxel centres sit at integer + 0.5.
    const Vec3f q = g - Vec3f::Constant(0.5f);
    const int x0 = static_cast<int>(std::floor(q.x())), y0 = static_cast<int>(std::floor(q.y())),
              z0 = static_cast<int>(std::floor(q.z()));
    const float fx = q.x() - static_cast<float>(x0), fy = q.y() - static_cast<float>(y0), fz = q.z() - static_cast<float>(z0);
    float acc = 0.0f;
    for (int dz = 0; dz < 2; ++dz)
        for (int dy = 0; dy < 2; ++dy)
            for (int dx = 0; dx < 2; ++dx) {
                float s, w;
                if (!r.get(x0 + dx, y0 + dy, z0 + dz, s, w)) return false;
                const float wx = dx ? fx : 1 - fx, wy = dy ? fy : 1 - fy, wz = dz ? fz : 1 - fz;
                acc += wx * wy * wz * s;
            }
    out = acc;
    return true;
}

// Trilinear where all 8 neighbours are observed, otherwise the nearest observed voxel.
bool robust_sample(VoxelReader& r, const Vec3f& g, float& out) {
    if (trilinear_sample(r, g, out)) return true;
    float w;
    return r.get(static_cast<int>(std::floor(g.x())), static_cast<int>(std::floor(g.y())), static_cast<int>(std::floor(g.z())),
                 out, w);
}

}  // namespace

void RaycastResult::ensure_cpu() const {
    if (!device || !points.empty()) return;
    const int w = device->width(), h = device->height();
    points = Image<Vec3f>(w, h, Vec3f::Zero());
    normals = Image<Vec3f>(w, h, Vec3f::Zero());
    valid = Image<std::uint8_t>(w, h, 0);
    const float* p = device->points_xyzw();
    const float* n = device->normals_xyzw();
    for (std::size_t i = 0; i < static_cast<std::size_t>(w * h); ++i) {
        if (p[4 * i + 3] == 0.0f) continue;
        points.data()[i] = Vec3f(p[4 * i], p[4 * i + 1], p[4 * i + 2]);
        normals.data()[i] = Vec3f(n[4 * i], n[4 * i + 1], n[4 * i + 2]);
        valid.data()[i] = 1;
    }
}

TsdfVolume::TsdfVolume(TsdfParams params) : params_(params), brick_mm_(params.voxel_mm * kBrickSize) {}

BrickCoord TsdfVolume::brick_of(const Vec3f& p) const {
    return {static_cast<int>(std::floor(p.x() / brick_mm_)), static_cast<int>(std::floor(p.y() / brick_mm_)),
            static_cast<int>(std::floor(p.z() / brick_mm_))};
}

const Brick* TsdfVolume::find(const BrickCoord& c) const {
    auto it = bricks_.find(c);
    return it == bricks_.end() ? nullptr : it->second.get();
}

void TsdfVolume::for_each_brick(const BrickVisitor& fn) const {
    std::shared_lock lock(mutex_);
    for (const auto& [c, b] : bricks_)
        fn(c, b->sdf, b->weight, params_.count_observations ? std::span<const std::uint8_t>(b->observations) : std::span<const std::uint8_t>{});
}

std::size_t TsdfVolume::brick_count() const {
    std::shared_lock lock(mutex_);
    return bricks_.size();
}

void TsdfVolume::clear() {
    std::unique_lock lock(mutex_);
    bricks_.clear();
    frame_counter_ = 0;
}

void TsdfVolume::integrate(const DepthFrame& frame, const SE3& T_world_camera, float weight_scale, bool extend_only) {
    frame.ensure_cpu();
    const Eigen::Matrix4f T_wc = T_world_camera.matrix().cast<float>();
    const Eigen::Matrix4f T_cw = T_world_camera.inverse().matrix().cast<float>();
    const Vec3f cam_center = T_wc.block<3, 1>(0, 3);
    const auto& k = frame.intrinsics;
    const float trunc = params_.truncation_mm;

    // 1. Bricks touched by the truncation band around every observed point.
    tbb::concurrent_vector<BrickCoord> touched;
    tbb::parallel_for(0, frame.points.height(), [&](int v) {
        BrickCoord last{std::numeric_limits<int>::min(), 0, 0};
        for (int u = 0; u < frame.points.width(); u += 1) {
            const Vec3f& pc = frame.points(u, v);
            if (pc.z() < params_.min_depth_mm || pc.z() > params_.max_depth_mm) continue;
            const Vec3f pw = (T_wc * pc.homogeneous()).head<3>();
            const Vec3f ray = (pw - cam_center).normalized();
            for (const float s : {-trunc, 0.0f, trunc}) {
                const BrickCoord c = brick_of(pw + s * ray);
                if (c == last) continue;
                last = c;
                touched.push_back(c);
            }
        }
    });
    std::unordered_set<BrickCoord, BrickCoordHash> unique(touched.begin(), touched.end());

    std::unique_lock lock(mutex_);
    ++frame_counter_;
    std::vector<Brick*> work;
    std::vector<BrickCoord> coords;
    work.reserve(unique.size());
    for (const auto& c : unique) {
        auto& slot = bricks_[c];
        if (!slot) slot = std::make_unique<Brick>();
        work.push_back(slot.get());
        coords.push_back(c);
    }

    // 2. Projective TSDF update of every voxel in the touched bricks.
    tbb::parallel_for(std::size_t{0}, work.size(), [&](std::size_t bi) {
        Brick& b = *work[bi];
        const BrickCoord c = coords[bi];
        bool updated = false;
        for (int z = 0; z < kBrickSize; ++z)
            for (int y = 0; y < kBrickSize; ++y)
                for (int x = 0; x < kBrickSize; ++x) {
                    const Vec3f pw((static_cast<float>(c.x * kBrickSize + x) + 0.5f) * params_.voxel_mm,
                                   (static_cast<float>(c.y * kBrickSize + y) + 0.5f) * params_.voxel_mm,
                                   (static_cast<float>(c.z * kBrickSize + z) + 0.5f) * params_.voxel_mm);
                    const Vec3f pc = (T_cw * pw.homogeneous()).head<3>();
                    if (pc.z() <= 0) continue;
                    const int u = static_cast<int>(std::lround(k.fx * pc.x() / pc.z() + k.cx));
                    const int v = static_cast<int>(std::lround(k.fy * pc.y() / pc.z() + k.cy));
                    if (u < 0 || v < 0 || u >= frame.points.width() || v >= frame.points.height()) continue;
                    const float zm = frame.points(u, v).z();
                    if (zm <= 0) continue;
                    const float sdf = zm - pc.z();
                    if (sdf < -trunc) continue;
                    const float tsdf = std::min(1.0f, sdf / trunc);
                    const float w_new = weight_scale * std::max(frame.weights.empty() ? 1.0f : frame.weights(u, v), 0.05f);
                    const auto i = static_cast<std::size_t>(voxel_index(x, y, z));
                    const float w_old = b.weight[i];
                    if (extend_only && w_old > 0.0f) continue;
                    b.sdf[i] = (b.sdf[i] * w_old + tsdf * w_new) / (w_old + w_new);
                    b.weight[i] = std::min(w_old + w_new, params_.max_weight);
                    if (params_.count_observations && std::abs(sdf) < trunc && b.observations[i] < 255) ++b.observations[i];
                    updated = true;
                }
        if (updated) b.last_update = frame_counter_;
    });
}

bool TsdfVolume::trilinear(const Vec3f& world, float& sdf) const {
    VoxelReader r{bricks_};
    return trilinear_sample(r, world / params_.voxel_mm, sdf);
}

std::optional<float> TsdfVolume::sample_sdf(const Vec3f& world) const {
    std::shared_lock lock(mutex_);
    float s;
    if (!trilinear(world, s)) return std::nullopt;
    return s * params_.truncation_mm;
}

RaycastResult TsdfVolume::raycast(const SE3& T_world_camera, const Intrinsics& k) const {
    std::shared_lock lock(mutex_);
    RaycastResult out{k, Image<Vec3f>(k.width, k.height, Vec3f::Zero()), Image<Vec3f>(k.width, k.height, Vec3f::Zero()),
                      Image<std::uint8_t>(k.width, k.height, 0)};
    const Eigen::Matrix4f T_cw = T_world_camera.inverse().matrix().cast<float>();
    const Eigen::Matrix3f R_wc = T_world_camera.linear().cast<float>();
    const Vec3f origin = T_world_camera.translation().cast<float>();

    // Per-tile depth bounds from projected bricks so rays start near the surface.
    constexpr int kTile = 16;
    const int tw = (k.width + kTile - 1) / kTile, th = (k.height + kTile - 1) / kTile;
    std::vector<float> tmin(static_cast<std::size_t>(tw * th), std::numeric_limits<float>::max());
    std::vector<float> tmax(static_cast<std::size_t>(tw * th), 0.0f);
    for (const auto& [c, brick] : bricks_) {
        float zmin = std::numeric_limits<float>::max(), zmax = 0;
        float umin = 1e9f, umax = -1e9f, vmin = 1e9f, vmax = -1e9f;
        bool any = false;
        for (int corner = 0; corner < 8; ++corner) {
            const Vec3f pw(static_cast<float>(c.x + (corner & 1)) * brick_mm_, static_cast<float>(c.y + ((corner >> 1) & 1)) * brick_mm_,
                           static_cast<float>(c.z + ((corner >> 2) & 1)) * brick_mm_);
            const Vec3f pc = (T_cw * pw.homogeneous()).head<3>();
            if (pc.z() < params_.min_depth_mm * 0.5f) continue;
            any = true;
            const float u = static_cast<float>(k.fx * pc.x() / pc.z() + k.cx), v = static_cast<float>(k.fy * pc.y() / pc.z() + k.cy);
            umin = std::min(umin, u), umax = std::max(umax, u), vmin = std::min(vmin, v), vmax = std::max(vmax, v);
            zmin = std::min(zmin, pc.z()), zmax = std::max(zmax, pc.z());
        }
        if (!any || umax < 0 || vmax < 0 || umin >= static_cast<float>(k.width) || vmin >= static_cast<float>(k.height)) continue;
        const int tx0 = std::max(0, static_cast<int>(umin) / kTile), tx1 = std::min(tw - 1, static_cast<int>(umax) / kTile);
        const int ty0 = std::max(0, static_cast<int>(vmin) / kTile), ty1 = std::min(th - 1, static_cast<int>(vmax) / kTile);
        for (int ty = ty0; ty <= ty1; ++ty)
            for (int tx = tx0; tx <= tx1; ++tx) {
                const auto i = static_cast<std::size_t>(ty * tw + tx);
                tmin[i] = std::min(tmin[i], zmin);
                tmax[i] = std::max(tmax[i], zmax);
            }
    }

    const float trunc = params_.truncation_mm;
    const float inv_voxel = 1.0f / params_.voxel_mm;
    tbb::parallel_for(0, k.height, [&](int v) {
        VoxelReader r{bricks_};
        for (int u = 0; u < k.width; ++u) {
            const auto ti = static_cast<std::size_t>((v / kTile) * tw + (u / kTile));
            if (tmax[ti] <= 0) continue;
            const Vec3f dir_c(static_cast<float>((u - k.cx) / k.fx), static_cast<float>((v - k.cy) / k.fy), 1.0f);
            const float dir_norm = dir_c.norm();
            const Vec3f dir = R_wc * (dir_c / dir_norm);
            // Convert camera-z bounds to ray distances.
            float t = std::max(params_.min_depth_mm, tmin[ti]) * dir_norm - trunc;
            const float t_end = std::min(params_.max_depth_mm, tmax[ti]) * dir_norm + trunc;
            float prev_sdf = 1.0f, prev_t = t;
            bool have_prev = false;
            while (t < t_end) {
                const Vec3f p = origin + t * dir;
                float s;
                if (!robust_sample(r, p * inv_voxel, s)) {
                    have_prev = false;
                    t += params_.voxel_mm;
                    continue;
                }
                if (have_prev && prev_sdf > 0 && s <= 0) {
                    const float t_hit = prev_t + (t - prev_t) * prev_sdf / (prev_sdf - s);
                    const Vec3f hit = origin + t_hit * dir;
                    // Gradient by central differences.
                    const Vec3f g = hit * inv_voxel;
                    float sx0, sx1, sy0, sy1, sz0, sz1;
                    if (robust_sample(r, g - Vec3f(1, 0, 0), sx0) && robust_sample(r, g + Vec3f(1, 0, 0), sx1) &&
                        robust_sample(r, g - Vec3f(0, 1, 0), sy0) && robust_sample(r, g + Vec3f(0, 1, 0), sy1) &&
                        robust_sample(r, g - Vec3f(0, 0, 1), sz0) && robust_sample(r, g + Vec3f(0, 0, 1), sz1)) {
                        Vec3f n(sx1 - sx0, sy1 - sy0, sz1 - sz0);
                        const float len = n.norm();
                        if (len > 1e-6f) {
                            out.points(u, v) = hit;
                            out.normals(u, v) = n / len;
                            out.valid(u, v) = 1;
                        }
                    }
                    break;
                }
                if (have_prev && prev_sdf < 0 && s < 0) break;  // started behind a surface
                prev_sdf = s;
                prev_t = t;
                have_prev = true;
                t += std::max(params_.voxel_mm * 0.5f, s * trunc * 0.8f);
            }
        }
    });
    return out;
}

std::vector<BrickCoord> TsdfVolume::bricks_updated_since(std::uint32_t frame) const {
    std::shared_lock lock(mutex_);
    std::vector<BrickCoord> out;
    for (const auto& [c, b] : bricks_)
        if (b->last_update > frame) out.push_back(c);
    return out;
}

void sort_canonical(std::vector<SurfacePoint>& points) {
    std::ranges::sort(points, [](const SurfacePoint& a, const SurfacePoint& b) {
        if (a.position.x() != b.position.x()) return a.position.x() < b.position.x();
        if (a.position.y() != b.position.y()) return a.position.y() < b.position.y();
        return a.position.z() < b.position.z();
    });
}

std::vector<SurfacePoint> TsdfVolume::extract_points(std::uint32_t since_frame, float min_weight, bool /*canonical: always*/) const {
    return extract_points(bricks_updated_since(since_frame), min_weight);
}

std::vector<SurfacePoint> TsdfVolume::extract_points(const std::vector<BrickCoord>& coords, float min_weight) const {
    std::shared_lock lock(mutex_);
    tbb::concurrent_vector<SurfacePoint> out;
    const float trunc = params_.truncation_mm;
    const float half_voxel_norm = 0.5f * params_.voxel_mm / trunc;
    tbb::parallel_for(std::size_t{0}, coords.size(), [&](std::size_t bi) {
        const BrickCoord c = coords[bi];
        const Brick* b = find(c);
        if (!b) return;
        VoxelReader r{bricks_};
        for (int z = 0; z < kBrickSize; ++z)
            for (int y = 0; y < kBrickSize; ++y)
                for (int x = 0; x < kBrickSize; ++x) {
                    const auto i = static_cast<std::size_t>(voxel_index(x, y, z));
                    if (b->weight[i] < min_weight || std::abs(b->sdf[i]) > half_voxel_norm) continue;
                    const int gx = c.x * kBrickSize + x, gy = c.y * kBrickSize + y, gz = c.z * kBrickSize + z;
                    float sx0, sx1, sy0, sy1, sz0, sz1, w;
                    if (!r.get(gx - 1, gy, gz, sx0, w) || !r.get(gx + 1, gy, gz, sx1, w) || !r.get(gx, gy - 1, gz, sy0, w) ||
                        !r.get(gx, gy + 1, gz, sy1, w) || !r.get(gx, gy, gz - 1, sz0, w) || !r.get(gx, gy, gz + 1, sz1, w))
                        continue;
                    Vec3f n(sx1 - sx0, sy1 - sy0, sz1 - sz0);
                    const float len = n.norm();
                    if (len < 1e-6f) continue;
                    n /= len;
                    const Vec3f center((static_cast<float>(gx) + 0.5f) * params_.voxel_mm, (static_cast<float>(gy) + 0.5f) * params_.voxel_mm,
                                       (static_cast<float>(gz) + 0.5f) * params_.voxel_mm);
                    out.push_back({center - n * (b->sdf[i] * trunc), n, b->weight[i]});
                }
    });
    return {out.begin(), out.end()};
}

}  // namespace einstar::track
