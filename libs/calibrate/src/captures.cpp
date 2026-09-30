#include "einstar/calibrate/captures.hpp"

#include <algorithm>
#include <cctype>
#include <format>
#include <fstream>
#include <iterator>

#include <tbb/parallel_for.h>

namespace einstar::calibrate {

namespace fs = std::filesystem;

std::optional<ImageU8> read_pgm(const fs::path& path) {
    std::ifstream f(path, std::ios::binary);
    std::string magic;
    int w = 0, h = 0, maxval = 0;
    f >> magic >> w >> h >> maxval;
    if (magic != "P5" || w <= 0 || h <= 0 || maxval != 255) return std::nullopt;
    f.get();
    ImageU8 img(w, h);
    f.read(reinterpret_cast<char*>(img.data()), static_cast<std::streamsize>(img.size()));
    if (!f) return std::nullopt;
    return img;
}

Result<void> write_pgm(const fs::path& path, ImageView<const std::uint8_t> image) {
    std::ofstream f(path, std::ios::binary);
    f << "P5\n" << image.width << ' ' << image.height << "\n255\n";
    for (int y = 0; y < image.height; ++y) f.write(reinterpret_cast<const char*>(image.row(y)), image.width);
    if (!f) return make_error(Errc::io, std::format("cannot write {}", path.string()));
    return {};
}

std::optional<ImageU8> read_bmp8(const fs::path& path) {
    std::ifstream f(path, std::ios::binary);
    std::vector<std::uint8_t> b((std::istreambuf_iterator<char>(f)), {});
    if (b.size() < 54 || b[0] != 'B' || b[1] != 'M') return std::nullopt;
    auto u32 = [&](std::size_t o) { return static_cast<std::uint32_t>(b[o] | b[o + 1] << 8 | b[o + 2] << 16 | b[o + 3] << 24); };
    const auto off = u32(10);
    const auto w = static_cast<int>(u32(18));
    const auto hs = static_cast<std::int32_t>(u32(22));
    const int bits = b[28] | b[29] << 8;
    const int h = std::abs(hs);
    const int stride = (w + 3) & ~3;
    if (bits != 8 || w <= 0 || h <= 0 || off + static_cast<std::size_t>(stride) * static_cast<std::size_t>(h) > b.size()) return std::nullopt;
    ImageU8 img(w, h);
    for (int y = 0; y < h; ++y) {
        const int src_row = hs > 0 ? h - 1 - y : y;  // bottom-up by default
        std::copy_n(b.begin() + off + static_cast<std::ptrdiff_t>(src_row) * stride, w, img.data() + static_cast<std::ptrdiff_t>(y) * w);
    }
    return img;
}

std::optional<ImageU8> read_image(const fs::path& path) {
    return path.extension() == ".bmp" ? read_bmp8(path) : read_pgm(path);
}

std::vector<CapturePair> list_capture_pairs(const fs::path& dir) {
    std::vector<std::pair<int, CapturePair>> found;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        const auto name = e.path().filename().string();
        const auto ext = e.path().extension().string();
        if (ext != ".pgm" && ext != ".bmp") continue;
        const auto stem = e.path().stem().string();
        CapturePair p;
        if (stem.starts_with("imageLeft")) {
            p.right = e.path().parent_path() / ("imageRight" + stem.substr(9) + ext);
        } else if (stem.ends_with("_s0")) {
            p.right = e.path().parent_path() / (stem.substr(0, stem.size() - 3) + "_s1" + ext);
        } else {
            continue;
        }
        if (!fs::exists(p.right)) continue;
        p.left = e.path();
        p.name = stem;
        std::string digits;
        for (const char c : stem)
            if (std::isdigit(static_cast<unsigned char>(c))) digits += c;
        found.emplace_back(digits.empty() ? 0 : std::stoi(digits.substr(0, 9)), std::move(p));
    }
    std::ranges::sort(found, [](const auto& a, const auto& b) { return a.first != b.first ? a.first < b.first : a.second.name < b.second.name; });
    std::vector<CapturePair> out;
    for (auto& f : found) out.push_back(std::move(f.second));
    return out;
}

Result<LoadedCaptures> load_captures(const fs::path& dir, const BoardSpec& board) {
    const auto pairs = list_capture_pairs(dir);
    if (pairs.empty()) return make_error(Errc::not_found, std::format("no imageLeftN / imageRightN pairs in {}", dir.string()));
    LoadedCaptures out;
    out.captures.resize(pairs.size());
    std::vector<int> widths(pairs.size(), 0), heights(pairs.size(), 0);
    tbb::parallel_for(std::size_t{0}, pairs.size(), [&](std::size_t i) {
        out.captures[i].name = pairs[i].name;
        const auto l = read_image(pairs[i].left), r = read_image(pairs[i].right);
        if (!l || !r) return;
        widths[i] = l->width(), heights[i] = l->height();
        if (auto d = detect_board(l->view(), board)) out.captures[i].left = std::move(*d);
        if (auto d = detect_board(r->view(), board)) out.captures[i].right = std::move(*d);
    });
    for (std::size_t i = 0; i < pairs.size(); ++i)
        if (widths[i] > 0) {
            out.width = widths[i], out.height = heights[i];
            break;
        }
    if (out.width == 0) return make_error(Errc::io, std::format("could not read the images in {}", dir.string()));
    return out;
}

}  // namespace einstar::calibrate
