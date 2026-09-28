#include "einstar/fixtures/exstar_project.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <map>

namespace einstar::fixtures {
namespace {

constexpr std::uint64_t kFirstFrameOffset = 0xA00000;

template <typename T>
T get(const std::uint8_t* p) {
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

Result<std::vector<std::uint8_t>> pread_all(int fd, std::uint64_t off, std::uint64_t n) {
    std::vector<std::uint8_t> buf(n);
    std::uint64_t done = 0;
    while (done < n) {
        const auto r = ::pread(fd, buf.data() + done, n - done, static_cast<off_t>(off + done));
        if (r <= 0) return make_error(Errc::io, std::format("read failed at offset {}", off + done));
        done += static_cast<std::uint64_t>(r);
    }
    return buf;
}

// Section table: key -> [begin, end) within the block.
Result<std::map<std::uint64_t, std::pair<std::size_t, std::size_t>>> sections(std::span<const std::uint8_t> block) {
    if (block.size() < 0x29 || std::memcmp(block.data(), "BLOCK", 5) != 0) return make_error(Errc::protocol, "not a BLOCK");
    const auto first = get<std::uint64_t>(block.data() + 0x21);
    const std::size_t n = (first - 1) / 8;
    std::vector<std::size_t> offs(n);
    for (std::size_t i = 0; i < n; ++i) offs[i] = 0x20 + get<std::uint64_t>(block.data() + 0x21 + 8 * i);
    std::map<std::uint64_t, std::pair<std::size_t, std::size_t>> out;
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t b = offs[i], e = i + 1 < n ? offs[i + 1] : block.size();
        if (b + 8 > block.size() || e > block.size() || e < b) return make_error(Errc::protocol, "bad section table");
        out[get<std::uint64_t>(block.data() + b)] = {b + 8, e};
    }
    return out;
}

MarkerObservation parse_marker(const std::uint8_t* p) {
    MarkerObservation m;
    m.position = Vec3(get<double>(p), get<double>(p + 8), get<double>(p + 16));
    m.normal = Vec3(get<double>(p + 24), get<double>(p + 32), get<double>(p + 40));
    m.observations = get<double>(p + 48);
    m.diameter = get<double>(p + 56);
    m.id = get<std::int32_t>(p + 72);
    return m;
}

constexpr std::size_t kMarkerRecord = 118;

}  // namespace

ExstarProject::~ExstarProject() {
    if (fd_ >= 0) ::close(fd_);
}

Result<std::unique_ptr<ExstarProject>> ExstarProject::open(const std::filesystem::path& path) {
    std::filesystem::path base = path;
    if (base.extension() == ".ir_E10_prj") base.replace_extension();
    auto proj = std::unique_ptr<ExstarProject>(new ExstarProject());

    // .data_cm: rectified full-res (cx, cy, f) at offset 0, baseline float at 660.
    {
        std::ifstream cm(base.string() + ".data_cm", std::ios::binary);
        std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(cm)), {});
        if (b.size() >= 664) {
            proj->fallback_ = {get<double>(b.data() + 16) / 2, get<double>(b.data() + 24) / 2, get<double>(b.data()) / 2,
                               get<double>(b.data() + 8) / 2};
            proj->baseline_ = get<float>(b.data() + 660);
        }
    }

    const auto data_path = base.string() + ".data_base";
    proj->fd_ = ::open(data_path.c_str(), O_RDONLY);
    if (proj->fd_ < 0) return make_error(Errc::not_found, std::format("cannot open {}", data_path));
    const auto file_size = static_cast<std::uint64_t>(std::filesystem::file_size(data_path));

    // Head block: global markers.
    if (auto hdr = pread_all(proj->fd_, 0, 0x20); hdr) {
        const auto head_size = get<std::uint64_t>(hdr->data() + 8);
        if (auto head = pread_all(proj->fd_, 0, head_size); head) {
            if (auto secs = sections(*head); secs) {
                for (const std::uint64_t key : {0x270ull, 0x288ull}) {
                    auto it = secs->find(key);
                    if (it == secs->end()) continue;
                    const auto [b, e] = it->second;
                    const auto count = get<std::uint64_t>(head->data() + b);
                    if (count == 0 || b + 8 + count * kMarkerRecord > e) continue;
                    std::vector<MarkerObservation> list;
                    for (std::uint64_t i = 0; i < count; ++i) list.push_back(parse_marker(head->data() + b + 8 + i * kMarkerRecord));
                    if (list.size() > proj->global_markers_.size()) proj->global_markers_ = std::move(list);
                }
            }
        }
    }

    // Index frame blocks.
    for (std::uint64_t off = kFirstFrameOffset; off + 0x20 <= file_size;) {
        auto hdr = pread_all(proj->fd_, off, 0x20);
        if (!hdr) return std::unexpected(hdr.error());
        if (std::memcmp(hdr->data(), "BLOCK", 5) != 0) return make_error(Errc::protocol, std::format("no BLOCK at {}", off));
        const auto size = get<std::uint64_t>(hdr->data() + 8);
        if (size < 0x29 || off + size > file_size) return make_error(Errc::protocol, "truncated frame block");
        proj->blocks_.push_back({off, size});
        off += size;
    }
    return proj;
}

Result<Frame> ExstarProject::read_frame(std::size_t index, bool apply_flags) const {
    if (index >= blocks_.size()) return make_error(Errc::invalid_argument, "frame index out of range");
    auto block = pread_all(fd_, blocks_[index].offset, blocks_[index].size);
    if (!block) return std::unexpected(block.error());
    auto secs = sections(*block);
    if (!secs) return std::unexpected(secs.error());
    const std::uint8_t* d = block->data();
    auto present = [&](std::uint64_t key) -> std::optional<std::size_t> {
        auto it = secs->find(key);
        if (it == secs->end() || it->second.second - it->second.first < 4 || get<std::uint32_t>(d + it->second.first) == 0)
            return std::nullopt;
        return it->second.first + 4;
    };

    Frame f;
    if (auto p = present(0x70)) f.id = get<std::int32_t>(d + *p);
    if (auto p = present(0x38)) {
        Mat3 R;
        for (int c = 0; c < 3; ++c)
            for (int r = 0; r < 3; ++r) R(r, c) = get<double>(d + *p + 8 * static_cast<std::size_t>(c * 3 + r));
        f.T_world_camera.linear() = R;
        f.T_world_camera.translation() = Vec3(get<double>(d + *p + 72), get<double>(d + *p + 80), get<double>(d + *p + 88));
    } else {
        return make_error(Errc::protocol, "frame has no pose");
    }
    if (auto p = present(0xA0)) {
        const std::uint8_t* v = d + *p;
        const int w = get<std::int32_t>(v), h = get<std::int32_t>(v + 4);
        f.intrinsics = {get<double>(v + 8), get<double>(v + 16), get<double>(v + 24), get<double>(v + 32)};
        if (f.intrinsics.fx == 0) f.intrinsics = fallback_;
        const auto n = get<std::uint64_t>(v + 40);
        if (w <= 0 || h <= 0 || n != static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h))
            return make_error(Errc::protocol, "bad depth image header");
        f.depth = ImageF32(w, h);
        std::memcpy(f.depth.data(), v + 48, n * 4);
        const std::uint8_t* after = v + 48 + 4 * n;
        const auto m = get<std::uint64_t>(after);
        const std::uint8_t* flags_hdr = after + 8 + 4 * m;
        const auto k = get<std::uint64_t>(flags_hdr);
        if (apply_flags && k == n) {
            const std::uint8_t* flags = flags_hdr + 8;
            for (std::uint64_t i = 0; i < n; ++i)
                if (flags[i]) f.depth.data()[i] = 0.0f;
        }
    }
    if (auto p = present(0x28)) {
        const auto count = get<std::uint64_t>(d + *p);
        for (std::uint64_t i = 0; i < count; ++i) f.markers.push_back(parse_marker(d + *p + 8 + i * kMarkerRecord));
    }
    return f;
}

Result<Mesh> load_stl(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return make_error(Errc::not_found, std::format("cannot open {}", path.string()));
    std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(f)), {});
    Mesh mesh;
    if (b.size() >= 84) {
        const auto n = get<std::uint32_t>(b.data() + 80);
        if (84 + static_cast<std::size_t>(n) * 50 == b.size()) {
            mesh.vertices.reserve(n * 3);
            for (std::uint32_t t = 0; t < n; ++t) {
                const std::uint8_t* p = b.data() + 84 + t * 50 + 12;
                for (int v = 0; v < 3; ++v)
                    mesh.vertices.emplace_back(get<float>(p + 12 * v), get<float>(p + 12 * v + 4), get<float>(p + 12 * v + 8));
            }
            return mesh;
        }
    }
    return make_error(Errc::unsupported, "only binary STL is supported");
}

namespace {

// Closest point on triangle (Ericson, Real-Time Collision Detection 5.1.5).
Vec3f closest_on_triangle(const Vec3f& p, const Vec3f& a, const Vec3f& b, const Vec3f& c) {
    const Vec3f ab = b - a, ac = c - a, ap = p - a;
    const float d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const Vec3f bp = p - b;
    const float d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
    const Vec3f cp = p - c;
    const float d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const float denom = 1.0f / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

}  // namespace

std::int64_t MeshDistance::key(int x, int y, int z) const {
    return (static_cast<std::int64_t>(x) & 0x1FFFFF) << 42 | (static_cast<std::int64_t>(y) & 0x1FFFFF) << 21 |
           (static_cast<std::int64_t>(z) & 0x1FFFFF);
}

MeshDistance::MeshDistance(const Mesh& mesh, float cell_mm) : mesh_(mesh), cell_(cell_mm) {
    for (std::uint32_t t = 0; t < mesh.vertices.size() / 3; ++t) {
        const Vec3f& a = mesh.vertices[3 * t];
        const Vec3f& b = mesh.vertices[3 * t + 1];
        const Vec3f& c = mesh.vertices[3 * t + 2];
        const Vec3f lo = a.cwiseMin(b).cwiseMin(c) / cell_, hi = a.cwiseMax(b).cwiseMax(c) / cell_;
        for (int z = static_cast<int>(std::floor(lo.z())); z <= static_cast<int>(std::floor(hi.z())); ++z)
            for (int y = static_cast<int>(std::floor(lo.y())); y <= static_cast<int>(std::floor(hi.y())); ++y)
                for (int x = static_cast<int>(std::floor(lo.x())); x <= static_cast<int>(std::floor(hi.x())); ++x)
                    grid_[key(x, y, z)].push_back(t);
    }
}

std::optional<float> MeshDistance::distance(const Vec3f& p, float max_mm) const {
    const int r = static_cast<int>(std::ceil(max_mm / cell_));
    const int cx = static_cast<int>(std::floor(p.x() / cell_)), cy = static_cast<int>(std::floor(p.y() / cell_)),
              cz = static_cast<int>(std::floor(p.z() / cell_));
    float best = max_mm * max_mm;
    bool found = false;
    for (int z = cz - r; z <= cz + r; ++z)
        for (int y = cy - r; y <= cy + r; ++y)
            for (int x = cx - r; x <= cx + r; ++x) {
                auto it = grid_.find(key(x, y, z));
                if (it == grid_.end()) continue;
                for (const auto t : it->second) {
                    const Vec3f q = closest_on_triangle(p, mesh_.vertices[3 * t], mesh_.vertices[3 * t + 1], mesh_.vertices[3 * t + 2]);
                    const float d2 = (q - p).squaredNorm();
                    if (d2 <= best) {
                        best = d2;
                        found = true;
                    }
                }
            }
    if (!found) return std::nullopt;
    return std::sqrt(best);
}

std::vector<Vec3f> unproject(const Frame& f, int step) {
    std::vector<Vec3f> out;
    const auto& k = f.intrinsics;
    for (int v = 0; v < f.depth.height(); v += step)
        for (int u = 0; u < f.depth.width(); u += step) {
            const float z = f.depth(u, v);
            if (z <= 0) continue;
            out.emplace_back(static_cast<float>((u - k.cx) * z / k.fx), static_cast<float>((v - k.cy) * z / k.fy), z);
        }
    return out;
}

}  // namespace einstar::fixtures
