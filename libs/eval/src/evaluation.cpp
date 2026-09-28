#include "einstar/eval/evaluation.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>

#include <Eigen/Geometry>

namespace einstar::eval {

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::ranges::sort(v);
    return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))];
}

namespace {
Stats stats(const std::vector<double>& v) { return {percentile(v, 0.5), percentile(v, 0.95)}; }
}  // namespace

void write_pose(std::ostream& out, std::size_t index, const SE3& T) {
    out << index;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) out << ' ' << std::setprecision(12) << T.matrix()(r, c);
    out << '\n';
}

std::map<std::size_t, SE3> read_poses(const std::filesystem::path& path) {
    std::ifstream f(path);
    std::map<std::size_t, SE3> out;
    std::size_t idx;
    while (f >> idx) {
        SE3 T = SE3::Identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) f >> T.matrix()(r, c);
        out[idx] = T;
    }
    return out;
}

PoseComparison compare_poses(const session::SessionReader& s, const std::map<std::size_t, SE3>& processed,
                             const std::map<std::size_t, SE3>& reference, double gross_mm) {
    PoseComparison c;
    std::vector<double> lt, lr, pt, prot, rec;
    for (const auto& [i, T] : processed) {
        const auto it = reference.find(s.meta(i).index);
        if (it == reference.end()) continue;
        const SE3 eo = it->second.inverse() * T;
        if (!s.meta(i).accepted()) {
            rec.push_back(translation_norm(eo));
            continue;
        }
        const SE3 el = it->second.inverse() * s.meta(i).T_world_camera;
        const bool lb = translation_norm(el) > gross_mm, ob = translation_norm(eo) > gross_mm;
        c.bad_processed_only += !lb && ob;
        c.bad_live_only += lb && !ob;
        c.bad_both += lb && ob;
        lt.push_back(translation_norm(el));
        lr.push_back(rotation_angle(el) * 180 / M_PI);
        pt.push_back(translation_norm(eo));
        prot.push_back(rotation_angle(eo) * 180 / M_PI);
    }
    c.frames = pt.size();
    c.live_mm = stats(lt), c.live_deg = stats(lr), c.processed_mm = stats(pt), c.processed_deg = stats(prot);
    c.recovered = rec.size();
    c.recovered_mm = stats(rec);
    c.recovered_bad = static_cast<int>(std::ranges::count_if(rec, [&](double e) { return e > gross_mm; }));
    auto aligned = [&](bool use_processed) {
        std::vector<Vec3> a, b;
        std::vector<SE3> ours, theirs;
        for (const auto& [i, T] : processed) {
            const auto it = reference.find(s.meta(i).index);
            if (it == reference.end() || !s.meta(i).accepted()) continue;
            const SE3 P = use_processed ? T : s.meta(i).T_world_camera;
            if (translation_norm(it->second.inverse() * P) > gross_mm) continue;  // gross (e.g. symmetric) cases
            a.push_back(P.translation());
            b.push_back(it->second.translation());
            ours.push_back(P);
            theirs.push_back(it->second);
        }
        if (a.size() < 3) return Stats{};
        Eigen::Matrix3Xd A(3, static_cast<Eigen::Index>(a.size())), B(3, static_cast<Eigen::Index>(b.size()));
        for (std::size_t n = 0; n < a.size(); ++n) A.col(static_cast<Eigen::Index>(n)) = a[n], B.col(static_cast<Eigen::Index>(n)) = b[n];
        SE3 G = SE3::Identity();
        G.matrix() = Eigen::umeyama(A, B, false);
        std::vector<double> e;
        for (std::size_t n = 0; n < ours.size(); ++n) e.push_back(translation_norm(theirs[n].inverse() * G * ours[n]));
        return stats(e);
    };
    c.aligned_live_mm = aligned(false);
    c.aligned_processed_mm = aligned(true);
    return c;
}

Stats marker_spread(const session::SessionReader& s, const std::function<std::optional<SE3>(std::size_t)>& pose_of) {
    std::map<int, std::vector<Vec3>> pts;
    for (std::size_t i = 0; i < s.frame_count(); ++i) {
        const auto P = pose_of(i);
        if (!P) continue;
        for (const auto& m : s.meta(i).markers)
            if (m.map_id >= 0) pts[m.map_id].push_back(*P * m.position);
    }
    std::vector<double> spread;
    for (const auto& [id, v] : pts) {
        if (v.size() < 5) continue;
        Vec3 c = Vec3::Zero();
        for (const auto& p : v) c += p;
        c /= static_cast<double>(v.size());
        for (const auto& p : v) spread.push_back((p - c).norm());
    }
    return stats(spread);
}

MeshComparison compare_mesh(const recon::TriangleMesh& mesh, const fixtures::Mesh& reference, float radius_mm) {
    MeshComparison c;
    const fixtures::MeshDistance to_ref(reference);
    fixtures::Mesh ours;
    ours.vertices.reserve(mesh.triangles.size() * 3);
    for (const auto& t : mesh.triangles)
        for (const auto v : t) ours.vertices.push_back(mesh.vertices[v]);
    const fixtures::MeshDistance to_ours(ours);
    std::vector<double> acc;
    int far = 0;
    const std::size_t step = std::max<std::size_t>(1, mesh.vertices.size() / 200000);
    for (std::size_t v = 0; v < mesh.vertices.size(); v += step) {
        if (auto d = to_ref.distance(mesh.vertices[v], radius_mm)) acc.push_back(*d);
        else ++far;
    }
    std::vector<double> comp;
    const std::size_t rstep = std::max<std::size_t>(1, reference.vertices.size() / 200000);
    for (std::size_t v = 0; v < reference.vertices.size(); v += rstep)
        comp.push_back(to_ours.distance(reference.vertices[v], radius_mm).value_or(radius_mm));
    const auto within = [](const std::vector<double>& d, double t) {
        return static_cast<double>(std::ranges::count_if(d, [&](double x) { return x <= t; })) / static_cast<double>(std::max<std::size_t>(1, d.size()));
    };
    c.accuracy_mm = stats(acc);
    c.accuracy_p90_mm = percentile(acc, 0.9);
    c.beyond_fraction = static_cast<double>(far) / static_cast<double>(std::max<std::size_t>(1, far + acc.size()));
    c.completeness_05 = within(comp, 0.5);
    c.completeness_1 = within(comp, 1.0);
    return c;
}

}  // namespace einstar::eval
