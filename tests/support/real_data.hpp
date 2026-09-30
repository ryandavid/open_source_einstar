#pragma once

// Real-sensor data for the tests, found in this order:
//   1. $EINSTAR_FIXTURES: a directory laid out like tests/fixtures/external;
//   2. tests/fixtures/external: excerpts packed with `einstar-cli fixture-pack` (see its README);
//   3. the original data: ~/Documents/EXStar/mustang_differential and EXStar's calibration captures.
// Packed fixtures are unpacked into a temporary directory once per test process.

#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "einstar/fixtures/exstar_project.hpp"
#include "einstar/fixtures/packaging.hpp"

namespace einstar::test_data {

namespace fs = std::filesystem;

inline std::vector<fs::path> packed_roots() {
    std::vector<fs::path> roots;
    if (const char* env = std::getenv("EINSTAR_FIXTURES"); env && *env) roots.emplace_back(env);
    roots.emplace_back(EINSTAR_TEST_FIXTURES_DIR);
    return roots;
}

// A packed fixture directory unpacked into a per-process temporary directory (removed at exit).
inline std::optional<fs::path> unpacked(const fs::path& packed) {
    static std::map<std::string, fs::path> cache;
    if (auto it = cache.find(packed.string()); it != cache.end()) return it->second;
    static const fs::path tmp_root = [] {
        auto p = fs::temp_directory_path() / std::format("einstar-fixtures-{}", ::getpid());
        std::atexit([] {
            std::error_code ec;
            fs::remove_all(fs::temp_directory_path() / std::format("einstar-fixtures-{}", ::getpid()), ec);
        });
        return p;
    }();
    const fs::path out = tmp_root / std::to_string(cache.size());
    if (!fixtures::unpack_directory(packed, out)) return std::nullopt;
    cache[packed.string()] = out;
    return out;
}

// The mustang_differential recording (EXStar project) or its packed excerpt.
struct Mustang {
    std::unique_ptr<fixtures::ExstarProject> project;
    std::size_t frame_count = 0;      // frames in the full recording
    std::vector<std::size_t> frames;  // excerpt: the recording's index of each frame present (empty: all)
    fs::path stl;                     // EXStar's mesh (cropped to the compared frames in the excerpt)

    // Frame by its index in the full recording.
    [[nodiscard]] Result<fixtures::Frame> read(std::size_t index) const {
        if (frames.empty()) return project->read_frame(index);
        const auto it = std::ranges::find(frames, index);
        if (it == frames.end()) return make_error(Errc::not_found, std::format("frame {} is not in the fixture excerpt", index));
        return project->read_frame(static_cast<std::size_t>(it - frames.begin()));
    }
};

inline std::optional<Mustang> mustang() {
    for (const auto& root : packed_roots()) {
        const fs::path packed = root / "mustang";
        if (!fs::exists(packed / "manifest.txt")) continue;
        const auto dir = unpacked(packed);
        if (!dir) return std::nullopt;
        Mustang m;
        std::ifstream manifest(*dir / "manifest.txt");
        for (std::string line; std::getline(manifest, line);) {
            std::istringstream in(line);
            std::string key;
            in >> key;
            if (key == "frame_count") in >> m.frame_count;
            if (key == "frames")
                for (std::size_t f; in >> f;) m.frames.push_back(f);
        }
        auto proj = fixtures::ExstarProject::open(*dir / "Project1.ir_E10_prj");
        if (!proj || (*proj)->frame_count() != m.frames.size()) return std::nullopt;
        m.project = std::move(*proj);
        m.stl = *dir / "mesh.stl";
        return m;
    }
    const char* home = std::getenv("HOME");
    const fs::path dir = fs::path(home ? home : "") / "Documents/EXStar/mustang_differential";
    if (!fs::exists(dir / "Project1.data_base")) return std::nullopt;
    auto proj = fixtures::ExstarProject::open(dir / "Project1.ir_E10_prj");
    if (!proj) return std::nullopt;
    Mustang m;
    m.frame_count = (*proj)->frame_count();
    m.project = std::move(*proj);
    m.stl = dir / "mustang_differential_simplified.stl";
    return m;
}

// Directory with EXStar's calibration-board captures (imageLeft<k>.bmp / imageRight<k>.bmp).
inline std::optional<fs::path> calibration_board() {
    for (const auto& root : packed_roots())
        if (fs::exists(root / "calibration_board" / "imageLeft1.bmp.zst")) return unpacked(root / "calibration_board");
    const fs::path exstar = "/Applications/EXStar.app/Contents/Frameworks/Applications/EXStar.app/Contents/Resources/res/Einscan-E10/calibrate_image_read_rapid";
    if (fs::exists(exstar / "imageLeft1.bmp")) return exstar;
    return std::nullopt;
}

}  // namespace einstar::test_data
