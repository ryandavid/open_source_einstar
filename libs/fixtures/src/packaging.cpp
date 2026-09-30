#include "einstar/fixtures/packaging.hpp"

#include <algorithm>
#include <cstring>
#include <format>
#include <fstream>

#include <zstd.h>

namespace einstar::fixtures {
namespace {

namespace fs = std::filesystem;

constexpr std::uint64_t kFirstFrameOffset = 0xA00000;  // (see exstar_project.cpp)

Result<void> copy_range(std::ifstream& in, std::uint64_t offset, std::uint64_t size, std::ofstream& out) {
    std::vector<char> buf(1 << 20);
    in.seekg(static_cast<std::streamoff>(offset));
    while (size > 0) {
        const auto n = static_cast<std::streamsize>(std::min<std::uint64_t>(size, buf.size()));
        if (!in.read(buf.data(), n)) return make_error(Errc::io, std::format("read failed at {}", offset));
        out.write(buf.data(), n);
        size -= static_cast<std::uint64_t>(n);
        offset += static_cast<std::uint64_t>(n);
    }
    if (!out) return make_error(Errc::io, "write failed");
    return {};
}

}  // namespace

std::vector<std::size_t> mustang_test_frames() {
    std::vector<std::size_t> f = mustang_mesh_frames();
    f.push_back(1);                                                // consecutive-frame ICP
    for (std::size_t i = 0; i < 480; i += 4) f.push_back(i);       // global registration model
    for (std::size_t i = 2; i < 480; i += 40) f.push_back(i);      // global registration probes
    std::ranges::sort(f);
    f.erase(std::ranges::unique(f).begin(), f.end());
    return f;
}

std::vector<std::size_t> mustang_mesh_frames() { return {0, 1500, 3000, 4500, 6000}; }

Result<void> write_project_subset(const fs::path& src, std::span<const std::size_t> frames, const fs::path& out_base) {
    fs::path base = src;
    if (base.extension() == ".ir_E10_prj") base.replace_extension();
    auto proj = ExstarProject::open(base);
    if (!proj) return std::unexpected(proj.error());
    std::ifstream in(base.string() + ".data_base", std::ios::binary);
    std::ofstream out(out_base.string() + ".data_base", std::ios::binary);
    if (!in || !out) return make_error(Errc::io, "cannot open the data files");
    if (auto r = copy_range(in, 0, kFirstFrameOffset, out); !r) return r;  // head: global markers
    for (const std::size_t i : frames) {
        if (i >= (*proj)->frame_count()) return make_error(Errc::invalid_argument, std::format("frame {} out of range", i));
        const auto [offset, size] = (*proj)->frame_block(i);
        if (auto r = copy_range(in, offset, size, out); !r) return r;
    }
    std::error_code ec;
    for (const char* ext : {".data_cm", ".ir_E10_prj"}) {
        fs::copy_file(base.string() + ext, out_base.string() + ext, fs::copy_options::overwrite_existing, ec);
        if (ec) return make_error(Errc::io, std::format("cannot copy {}{}: {}", base.string(), ext, ec.message()));
    }
    return {};
}

Mesh crop(const Mesh& mesh, const Vec3f& lo, const Vec3f& hi) {
    Mesh out;
    auto inside = [&](const Vec3f& v) { return (v.array() >= lo.array()).all() && (v.array() <= hi.array()).all(); };
    for (std::size_t t = 0; t + 2 < mesh.vertices.size(); t += 3)
        if (inside(mesh.vertices[t]) || inside(mesh.vertices[t + 1]) || inside(mesh.vertices[t + 2]))
            out.vertices.insert(out.vertices.end(), mesh.vertices.begin() + static_cast<std::ptrdiff_t>(t),
                                mesh.vertices.begin() + static_cast<std::ptrdiff_t>(t + 3));
    return out;
}

Result<void> write_stl(const Mesh& mesh, const fs::path& path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return make_error(Errc::io, std::format("cannot write {}", path.string()));
    char header[80] = "einstar test fixture";
    f.write(header, 80);
    const auto n = static_cast<std::uint32_t>(mesh.vertices.size() / 3);
    f.write(reinterpret_cast<const char*>(&n), 4);
    for (std::uint32_t t = 0; t < n; ++t) {
        const Vec3f& a = mesh.vertices[3 * t];
        const Vec3f& b = mesh.vertices[3 * t + 1];
        const Vec3f& c = mesh.vertices[3 * t + 2];
        const Vec3f nrm = (b - a).cross(c - a).normalized();
        for (const Vec3f& v : {nrm, a, b, c}) f.write(reinterpret_cast<const char*>(v.data()), 12);
        const std::uint16_t attr = 0;
        f.write(reinterpret_cast<const char*>(&attr), 2);
    }
    if (!f) return make_error(Errc::io, std::format("write failed: {}", path.string()));
    return {};
}

Result<void> compress_file(const fs::path& in_path, const fs::path& out_path, int level) {
    std::ifstream in(in_path, std::ios::binary);
    std::ofstream out(out_path, std::ios::binary);
    if (!in || !out) return make_error(Errc::io, std::format("cannot compress {}", in_path.string()));
    ZSTD_CCtx* cctx = ZSTD_createCCtx();
    ZSTD_CCtx_setParameter(cctx, ZSTD_c_compressionLevel, level);
    ZSTD_CCtx_setParameter(cctx, ZSTD_c_checksumFlag, 1);
    std::vector<char> ibuf(ZSTD_CStreamInSize()), obuf(ZSTD_CStreamOutSize());
    bool ok = true;
    for (;;) {
        in.read(ibuf.data(), static_cast<std::streamsize>(ibuf.size()));
        const auto got = static_cast<std::size_t>(in.gcount());
        const bool last = got < ibuf.size();
        ZSTD_inBuffer input{ibuf.data(), got, 0};
        for (bool done = false; !done;) {
            ZSTD_outBuffer output{obuf.data(), obuf.size(), 0};
            const std::size_t rem = ZSTD_compressStream2(cctx, &output, &input, last ? ZSTD_e_end : ZSTD_e_continue);
            if (ZSTD_isError(rem)) {
                ok = false;
                break;
            }
            out.write(obuf.data(), static_cast<std::streamsize>(output.pos));
            done = last ? rem == 0 : input.pos == input.size;
        }
        if (!ok || last) break;
    }
    ZSTD_freeCCtx(cctx);
    if (!ok || !out) return make_error(Errc::io, std::format("zstd compression of {} failed", in_path.string()));
    return {};
}

Result<void> decompress_file(const fs::path& in_path, const fs::path& out_path) {
    std::ifstream in(in_path, std::ios::binary);
    std::ofstream out(out_path, std::ios::binary);
    if (!in || !out) return make_error(Errc::io, std::format("cannot decompress {}", in_path.string()));
    ZSTD_DCtx* dctx = ZSTD_createDCtx();
    std::vector<char> ibuf(ZSTD_DStreamInSize()), obuf(ZSTD_DStreamOutSize());
    bool ok = true;
    std::size_t last_ret = 0;
    while (ok) {
        in.read(ibuf.data(), static_cast<std::streamsize>(ibuf.size()));
        const auto got = static_cast<std::size_t>(in.gcount());
        if (got == 0) break;
        ZSTD_inBuffer input{ibuf.data(), got, 0};
        while (input.pos < input.size) {
            ZSTD_outBuffer output{obuf.data(), obuf.size(), 0};
            last_ret = ZSTD_decompressStream(dctx, &output, &input);
            if (ZSTD_isError(last_ret)) {
                ok = false;
                break;
            }
            out.write(obuf.data(), static_cast<std::streamsize>(output.pos));
        }
    }
    ZSTD_freeDCtx(dctx);
    if (!ok || last_ret != 0 || !out) return make_error(Errc::io, std::format("{} is not a complete zstd file", in_path.string()));
    return {};
}

Result<void> unpack_directory(const fs::path& dir, const fs::path& out) {
    std::error_code ec;
    fs::create_directories(out, ec);
    if (ec) return make_error(Errc::io, std::format("cannot create {}: {}", out.string(), ec.message()));
    for (const auto& e : fs::directory_iterator(dir)) {
        if (!e.is_regular_file()) continue;
        const auto& p = e.path();
        if (p.extension() == ".zst") {
            if (auto r = decompress_file(p, out / p.stem()); !r) return r;
        } else {
            fs::copy_file(p, out / p.filename(), fs::copy_options::overwrite_existing, ec);
            if (ec) return make_error(Errc::io, std::format("cannot copy {}: {}", p.string(), ec.message()));
        }
    }
    return {};
}

}  // namespace einstar::fixtures
