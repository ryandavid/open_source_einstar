#include "einstar/model/io.hpp"

#include <bit>
#include <cstring>
#include <format>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include <zstd.h>

#include "einstar/model/json.hpp"
#include "einstar/recon/process.hpp"
#include "einstar/session/session.hpp"

namespace einstar::model {
namespace {

std::int64_t mtime_of(const std::filesystem::path& p) {
    std::error_code ec;
    const auto t = std::filesystem::last_write_time(p, ec);
    return ec ? 0 : static_cast<std::int64_t>(t.time_since_epoch().count());
}

// Triangle soup to an indexed mesh: corners at the same position become one vertex.
recon::TriangleMesh weld(const std::vector<Vec3f>& corners) {
    struct Hash {
        std::size_t operator()(const Vec3f& v) const noexcept {
            return std::bit_cast<std::uint32_t>(v.x()) * 73856093u ^ std::bit_cast<std::uint32_t>(v.y()) * 19349669u ^
                   std::bit_cast<std::uint32_t>(v.z()) * 83492791u;
        }
    };
    struct Eq {
        bool operator()(const Vec3f& a, const Vec3f& b) const noexcept { return a == b; }
    };
    recon::TriangleMesh m;
    std::unordered_map<Vec3f, std::uint32_t, Hash, Eq> index;
    index.reserve(corners.size() / 3);
    const auto vertex = [&](const Vec3f& p) {
        const auto [it, added] = index.try_emplace(p, static_cast<std::uint32_t>(m.vertices.size()));
        if (added) m.vertices.push_back(p);
        return it->second;
    };
    for (std::size_t i = 0; i + 2 < corners.size(); i += 3) {
        const std::array<std::uint32_t, 3> t{vertex(corners[i]), vertex(corners[i + 1]), vertex(corners[i + 2])};
        if (t[0] != t[1] && t[1] != t[2] && t[0] != t[2]) m.triangles.push_back(t);
    }
    m.compute_normals();
    return m;
}

Result<recon::TriangleMesh> read_stl(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return make_error(Errc::io, "cannot read " + path.string());
    const std::string data((std::istreambuf_iterator<char>(f)), {});
    std::vector<Vec3f> corners;
    if (data.size() >= 84) {
        std::uint32_t n = 0;
        std::memcpy(&n, data.data() + 80, 4);
        if (data.size() == 84 + std::size_t{n} * 50) {
            corners.reserve(std::size_t{n} * 3);
            for (std::size_t t = 0; t < n; ++t)
                for (int c = 0; c < 3; ++c) {
                    std::array<float, 3> v{};
                    std::memcpy(v.data(), data.data() + 84 + t * 50 + 12 + static_cast<std::size_t>(c) * 12, 12);
                    corners.emplace_back(v[0], v[1], v[2]);
                }
            return weld(corners);
        }
    }
    // ASCII: "vertex x y z" lines.
    std::istringstream in(data);
    std::string word;
    while (in >> word)
        if (word == "vertex") {
            Vec3f v;
            in >> v.x() >> v.y() >> v.z();
            corners.push_back(v);
        }
    if (corners.empty()) return make_error(Errc::invalid_argument, "no triangles in " + path.string());
    return weld(corners);
}

Result<recon::TriangleMesh> read_ply(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return make_error(Errc::io, "cannot read " + path.string());
    std::string line, format;
    std::size_t vertices = 0, faces = 0;
    std::vector<std::string> vertex_props;
    std::string current;
    std::string face_count_type = "uchar", face_index_type = "uint";
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream ls(line);
        std::string w;
        ls >> w;
        if (w == "format") ls >> format;
        else if (w == "element") {
            ls >> current;
            std::size_t n = 0;
            ls >> n;
            (current == "vertex" ? vertices : current == "face" ? faces : n) = n;
        } else if (w == "property" && current == "vertex") {
            std::string type, name;
            ls >> type >> name;
            if (type != "float") return make_error(Errc::unsupported, "PLY vertex properties must be float: " + path.string());
            vertex_props.push_back(name);
        } else if (w == "property" && current == "face") {
            std::string list, name;
            ls >> list >> face_count_type >> face_index_type >> name;
        } else if (w == "end_header") break;
    }
    const auto prop = [&](const char* name) {
        const auto it = std::ranges::find(vertex_props, name);
        return it == vertex_props.end() ? -1 : static_cast<int>(it - vertex_props.begin());
    };
    const int ix = prop("x"), iy = prop("y"), iz = prop("z"), inx = prop("nx"), iny = prop("ny"), inz = prop("nz");
    if (ix < 0 || iy < 0 || iz < 0) return make_error(Errc::invalid_argument, "PLY without x, y, z: " + path.string());
    recon::TriangleMesh m;
    m.vertices.resize(vertices);
    const bool normals = inx >= 0 && iny >= 0 && inz >= 0;
    if (normals) m.normals.resize(vertices);
    std::vector<float> row(vertex_props.size());
    if (format == "binary_little_endian") {
        if (face_count_type != "uchar" || (face_index_type != "uint" && face_index_type != "int"))
            return make_error(Errc::unsupported, "PLY faces must be 'list uchar uint' or 'list uchar int': " + path.string());
        for (std::size_t v = 0; v < vertices; ++v) {
            f.read(reinterpret_cast<char*>(row.data()), static_cast<std::streamsize>(row.size() * sizeof(float)));
            m.vertices[v] = {row[static_cast<std::size_t>(ix)], row[static_cast<std::size_t>(iy)], row[static_cast<std::size_t>(iz)]};
            if (normals) m.normals[v] = {row[static_cast<std::size_t>(inx)], row[static_cast<std::size_t>(iny)], row[static_cast<std::size_t>(inz)]};
        }
        for (std::size_t t = 0; t < faces; ++t) {
            std::uint8_t n = 0;
            f.read(reinterpret_cast<char*>(&n), 1);
            std::vector<std::uint32_t> idx(n);
            f.read(reinterpret_cast<char*>(idx.data()), static_cast<std::streamsize>(n * sizeof(std::uint32_t)));
            for (std::size_t k = 2; k < n; ++k) m.triangles.push_back({idx[0], idx[k - 1], idx[k]});
        }
        if (!f) return make_error(Errc::io, "PLY ends early: " + path.string());
    } else if (format == "ascii") {
        for (std::size_t v = 0; v < vertices; ++v) {
            for (auto& x : row) f >> x;
            m.vertices[v] = {row[static_cast<std::size_t>(ix)], row[static_cast<std::size_t>(iy)], row[static_cast<std::size_t>(iz)]};
            if (normals) m.normals[v] = {row[static_cast<std::size_t>(inx)], row[static_cast<std::size_t>(iny)], row[static_cast<std::size_t>(inz)]};
        }
        for (std::size_t t = 0; t < faces; ++t) {
            std::size_t n = 0;
            f >> n;
            std::vector<std::uint32_t> idx(n);
            for (auto& i : idx) f >> i;
            for (std::size_t k = 2; k < n; ++k) m.triangles.push_back({idx[0], idx[k - 1], idx[k]});
        }
    } else {
        return make_error(Errc::unsupported, std::format("PLY format '{}' is not supported", format));
    }
    for (const auto& t : m.triangles)
        for (const auto i : t)
            if (i >= m.vertices.size()) return make_error(Errc::invalid_argument, "PLY face refers to a missing vertex: " + path.string());
    if (!normals) m.compute_normals();
    return m;
}

// ---- project records ----

constexpr std::array<char, 4> kMagic{'E', 'M', 'D', 'L'};
constexpr std::uint32_t kVersion = 1;

std::uint32_t tag(const char (&t)[5]) { return static_cast<std::uint32_t>(t[0]) | static_cast<std::uint32_t>(t[1]) << 8 | static_cast<std::uint32_t>(t[2]) << 16 | static_cast<std::uint32_t>(t[3]) << 24; }

void put_record(std::ofstream& f, std::uint32_t t, const std::string& payload) {
    const std::uint32_t reserved = 0;
    const std::uint64_t size = payload.size();
    f.write(reinterpret_cast<const char*>(&t), 4);
    f.write(reinterpret_cast<const char*>(&reserved), 4);
    f.write(reinterpret_cast<const char*>(&size), 8);
    f.write(payload.data(), static_cast<std::streamsize>(payload.size()));
}

std::string compress(const void* data, std::size_t size) {
    std::string out(ZSTD_compressBound(size) + 8, '\0');
    const std::uint64_t raw = size;
    std::memcpy(out.data(), &raw, 8);
    const std::size_t n = ZSTD_compress(out.data() + 8, out.size() - 8, data, size, 3);
    out.resize(ZSTD_isError(n) ? 0 : n + 8);
    return out;
}

bool decompress(const std::string& in, void* data, std::size_t size) {
    if (in.size() < 8) return false;
    std::uint64_t raw = 0;
    std::memcpy(&raw, in.data(), 8);
    if (raw != size) return false;
    const std::size_t n = ZSTD_decompress(data, size, in.data() + 8, in.size() - 8);
    return !ZSTD_isError(n) && n == size;
}

template <class T>
std::string pack(const std::vector<T>& v) {
    return compress(v.data(), v.size() * sizeof(T));
}

json label_to_json(const Label& l) {
    json j = {{"id", l.id}, {"name", l.name}, {"role", role_name(l.role)}, {"sigma", l.sigma}, {"rms", l.rms}};
    j["kinds"] = json::array();
    for (const auto k : l.kinds) j["kinds"].push_back(fit::kind_name(k));
    if (l.fit) j["fit"] = surface_to_json(*l.fit);
    if (l.given) j["given"] = surface_to_json(*l.given);
    return j;
}

Label label_from_json(const json& j) {
    Label l;
    l.id = j.at("id");
    l.name = j.at("name");
    l.role = role_from_name(j.at("role").get<std::string>()).value_or(Role::face);
    l.sigma = j.value("sigma", 0.0);
    l.rms = j.value("rms", 0.0);
    l.kinds = kinds_from_json(j.value("kinds", json::array()));
    if (j.contains("fit")) l.fit = surface_from_json(j["fit"]);
    if (j.contains("given")) l.given = surface_from_json(j["given"]);
    return l;
}

}  // namespace

Result<std::pair<recon::TriangleMesh, ScanSource>> load_scan_mesh(const std::filesystem::path& path, bool fine) {
    std::error_code ec;
    if (!std::filesystem::exists(path, ec)) return make_error(Errc::not_found, "no such file: " + path.string());
    ScanSource src;
    src.path = std::filesystem::absolute(path, ec);
    src.size = std::filesystem::file_size(path, ec);
    src.mtime = mtime_of(path);
    std::string ext = path.extension().string();
    std::ranges::transform(ext, ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".stl" || ext == ".ply") {
        auto m = ext == ".stl" ? read_stl(path) : read_ply(path);
        if (!m) return std::unexpected(m.error());
        src.kind = ext.substr(1);
        return std::pair{std::move(*m), std::move(src)};
    }
    if (ext == ".estr") {
        auto reader = session::SessionReader::open(path.string());
        if (!reader) return std::unexpected(reader.error());
        recon::ProcessParams params;
        if (fine) params.tsdf.voxel_mm = 0.3f;
        auto r = recon::process_session(**reader, params);
        if (!r) return std::unexpected(r.error());
        src.kind = "estr";
        src.process = {{"voxel_mm", params.tsdf.voxel_mm}};
        return std::pair{std::move(r->mesh), std::move(src)};
    }
    return make_error(Errc::unsupported, "open a .estr, .stl, .ply or .emodel file: " + path.string());
}

Result<void> Document::save(const std::filesystem::path& path) const {
    if (!has_scan()) return make_error(Errc::invalid_argument, "no scan to save");
    const std::filesystem::path tmp = path.string() + ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary);
        if (!f) return make_error(Errc::io, "cannot write " + path.string());
        f.write(kMagic.data(), 4);
        f.write(reinterpret_cast<const char*>(&kVersion), 4);
        std::error_code ec;
        const json scan = {{"path", source_.path.string()},
                           {"relative", source_.path.empty() ? "" : std::filesystem::relative(source_.path, path.parent_path(), ec).string()},
                           {"kind", source_.kind},
                           {"size", source_.size},
                           {"mtime", source_.mtime},
                           {"process", source_.process}};
        put_record(f, tag("SCAN"), scan.dump());
        const auto& m = *mesh_;
        json mesh_header = {{"vertices", m.vertices.size()}, {"triangles", m.triangles.size()}};
        std::string mesh_payload = mesh_header.dump();
        mesh_payload.push_back('\0');
        std::vector<float> floats;
        floats.reserve(m.vertices.size() * 6);
        for (std::size_t v = 0; v < m.vertices.size(); ++v) {
            floats.insert(floats.end(), {m.vertices[v].x(), m.vertices[v].y(), m.vertices[v].z()});
            const Vec3f n = v < m.normals.size() ? m.normals[v] : Vec3f::Zero();
            floats.insert(floats.end(), {n.x(), n.y(), n.z()});
        }
        std::vector<std::uint32_t> idx;
        idx.reserve(m.triangles.size() * 3);
        for (const auto& t : m.triangles) idx.insert(idx.end(), t.begin(), t.end());
        const std::string fv = pack(floats), iv = pack(idx);
        const std::uint64_t fsz = fv.size();
        mesh_payload.append(reinterpret_cast<const char*>(&fsz), 8);
        mesh_payload += fv;
        mesh_payload += iv;
        put_record(f, tag("MESH"), mesh_payload);
        std::vector<std::uint16_t> labels(state_.paint->begin(), state_.paint->end());
        labels.insert(labels.end(), state_.region->begin(), state_.region->end());
        put_record(f, tag("LABL"), pack(labels));
        json doc = {{"next_id", state_.next_id}, {"labels", json::array()}, {"holes", json::array()}, {"fillets", json::array()},
                    {"datums", json::array()}, {"constraints", json::array()}, {"solved", json::object()}, {"solve_report", state_.solve_report}};
        for (const auto& l : state_.labels) doc["labels"].push_back(label_to_json(l));
        for (const auto& h : state_.holes) {
            json j = {{"id", h.id}, {"name", h.name}, {"host", h.host}, {"center", vec_to_json(h.center)}, {"axis", {h.axis.x(), h.axis.y(), h.axis.z()}},
                      {"measured_diameter", h.measured_diameter}, {"wall_seen", h.wall_seen}, {"wall_label", h.wall_label},
                      {"seen_depth", h.seen_depth}};
            if (h.diameter) j["diameter"] = *h.diameter;
            if (h.depth) j["depth"] = *h.depth;
            doc["holes"].push_back(j);
        }
        for (const auto& fl : state_.fillets) {
            json j = {{"id", fl.id}, {"name", fl.name}, {"label", fl.label}, {"a", fl.face_a}, {"b", fl.face_b}, {"measured_radius", fl.measured_radius}};
            if (fl.radius) j["radius"] = *fl.radius;
            doc["fillets"].push_back(j);
        }
        for (const auto& x : state_.datums) doc["datums"].push_back({{"id", x.id}, {"name", x.name}, {"frame", frame_to_json(x.frame)}});
        for (const auto& c : state_.constraints) doc["constraints"].push_back({{"id", c.id}, {"spec", c.spec}});
        for (const auto& [id, s] : state_.solved) doc["solved"][std::to_string(id)] = surface_to_json(s);
        put_record(f, tag("DOCU"), doc.dump());
        if (!f) return make_error(Errc::io, "cannot write " + path.string());
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) return make_error(Errc::io, "cannot write " + path.string() + ": " + ec.message());
    return {};
}

Result<void> Document::load(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return make_error(Errc::io, "cannot read " + path.string());
    std::array<char, 4> magic{};
    std::uint32_t version = 0;
    f.read(magic.data(), 4);
    f.read(reinterpret_cast<char*>(&version), 4);
    if (magic != kMagic) return make_error(Errc::invalid_argument, "not an .emodel file: " + path.string());
    if (version > kVersion) return make_error(Errc::unsupported, "made by a newer version: " + path.string());
    std::map<std::uint32_t, std::string> records;
    while (f) {
        std::uint32_t t = 0, reserved = 0;
        std::uint64_t size = 0;
        if (!f.read(reinterpret_cast<char*>(&t), 4) || !f.read(reinterpret_cast<char*>(&reserved), 4) || !f.read(reinterpret_cast<char*>(&size), 8)) break;
        std::string payload(size, '\0');
        if (!f.read(payload.data(), static_cast<std::streamsize>(size))) return make_error(Errc::io, "the file ends early: " + path.string());
        records[t] = std::move(payload);
    }
    for (const char* need : {"SCAN", "MESH", "LABL", "DOCU"}) {
        char t[5] = {need[0], need[1], need[2], need[3], 0};
        if (!records.contains(tag(t))) return make_error(Errc::invalid_argument, std::format("{} has no {} record", path.string(), need));
    }
    try {
        const json scan = json::parse(records[tag("SCAN")]);
        ScanSource src{scan.value("path", ""), scan.value("kind", ""), scan.value("size", std::uintmax_t{0}), scan.value("mtime", std::int64_t{0}),
                       scan.value("process", json::object())};
        // The mesh.
        const std::string& mp = records[tag("MESH")];
        const auto nul = mp.find('\0');
        const json header = json::parse(mp.substr(0, nul));
        const std::size_t nv = header.at("vertices"), nt = header.at("triangles");
        std::uint64_t fsz = 0;
        std::memcpy(&fsz, mp.data() + nul + 1, 8);
        const std::string fv = mp.substr(nul + 9, fsz), iv = mp.substr(nul + 9 + fsz);
        std::vector<float> floats(nv * 6);
        std::vector<std::uint32_t> idx(nt * 3);
        if (!decompress(fv, floats.data(), floats.size() * sizeof(float)) || !decompress(iv, idx.data(), idx.size() * sizeof(std::uint32_t)))
            return make_error(Errc::invalid_argument, "damaged mesh in " + path.string());
        recon::TriangleMesh m;
        m.vertices.resize(nv);
        m.normals.resize(nv);
        for (std::size_t v = 0; v < nv; ++v) {
            m.vertices[v] = {floats[v * 6], floats[v * 6 + 1], floats[v * 6 + 2]};
            m.normals[v] = {floats[v * 6 + 3], floats[v * 6 + 4], floats[v * 6 + 5]};
        }
        m.triangles.resize(nt);
        for (std::size_t t = 0; t < nt; ++t) m.triangles[t] = {idx[t * 3], idx[t * 3 + 1], idx[t * 3 + 2]};
        std::vector<std::uint16_t> labels(nt * 2);
        if (!decompress(records[tag("LABL")], labels.data(), labels.size() * sizeof(std::uint16_t)))
            return make_error(Errc::invalid_argument, "damaged labels in " + path.string());
        const json doc = json::parse(records[tag("DOCU")]);

        set_scan(std::move(m), std::move(src));
        State s;
        s.paint = std::make_shared<const std::vector<std::uint16_t>>(labels.begin(), labels.begin() + static_cast<std::ptrdiff_t>(nt));
        s.region = std::make_shared<const std::vector<std::uint16_t>>(labels.begin() + static_cast<std::ptrdiff_t>(nt), labels.end());
        s.next_id = doc.at("next_id");
        for (const auto& j : doc.at("labels")) s.labels.push_back(label_from_json(j));
        for (const auto& j : doc.at("holes")) {
            Hole h;
            h.id = j.at("id");
            h.name = j.at("name");
            h.host = j.at("host");
            h.center = vec_from_json(j.at("center"));
            h.axis = vec_from_json(j.at("axis")).normalized();
            h.measured_diameter = j.at("measured_diameter");
            h.wall_seen = j.value("wall_seen", false);
            h.wall_label = j.value("wall_label", 0);
            h.seen_depth = j.value("seen_depth", 0.0);
            if (j.contains("diameter")) h.diameter = j["diameter"].get<double>();
            if (j.contains("depth")) h.depth = j["depth"].get<double>();
            s.holes.push_back(h);
        }
        for (const auto& j : doc.at("fillets")) {
            Fillet fl;
            fl.id = j.at("id");
            fl.name = j.at("name");
            fl.label = j.value("label", 0);
            fl.face_a = j.at("a");
            fl.face_b = j.at("b");
            fl.measured_radius = j.at("measured_radius");
            if (j.contains("radius")) fl.radius = j["radius"].get<double>();
            s.fillets.push_back(fl);
        }
        for (const auto& j : doc.at("datums")) s.datums.push_back({j.at("id"), j.at("name"), frame_from_json(j.at("frame"))});
        for (const auto& j : doc.at("constraints")) s.constraints.push_back({j.at("id"), j.at("spec")});
        for (const auto& [k, v] : doc.at("solved").items()) s.solved[std::stoi(k)] = surface_from_json(v);
        s.solve_report = doc.value("solve_report", json::object());
        state_ = std::move(s);
        path_ = path;
        ++revision_;
    } catch (const json::exception& e) {
        return make_error(Errc::invalid_argument, std::format("damaged document in {}: {}", path.string(), e.what()));
    }
    return {};
}

}  // namespace einstar::model
