#pragma once

// Calibration capture folders. einstar-calibrate writes imageLeftN.pgm / imageRightN.pgm (and
// imageTexN.pgm, the colour camera's raw Bayer) like EXStar's calibration folders (imageLeftN.bmp),
// so either can be solved and compared with `einstar-cli calib-solve`. einstar-cli hw-capture's
// gNNN_s0.pgm / gNNN_s1.pgm pairs are read too.

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "einstar/calibrate/solve.hpp"
#include "einstar/core/error.hpp"
#include "einstar/core/image.hpp"

namespace einstar::calibrate {

[[nodiscard]] std::optional<ImageU8> read_pgm(const std::filesystem::path& path);
[[nodiscard]] Result<void> write_pgm(const std::filesystem::path& path, ImageView<const std::uint8_t> image);
[[nodiscard]] std::optional<ImageU8> read_bmp8(const std::filesystem::path& path);  // 8-bit palettised (EXStar)
// .pgm or .bmp by extension.
[[nodiscard]] std::optional<ImageU8> read_image(const std::filesystem::path& path);

struct CapturePair {
    std::string name;  // e.g. "imageLeft7"
    std::filesystem::path left, right;
};
// Left/right image pairs in a folder, in capture order.
[[nodiscard]] std::vector<CapturePair> list_capture_pairs(const std::filesystem::path& dir);

// Detects the board in every pair (in parallel). Pairs without the board in both images are kept with
// whatever was found (solve_stereo skips them).
struct LoadedCaptures {
    std::vector<StereoCapture> captures;
    int width = 0, height = 0;
};
[[nodiscard]] Result<LoadedCaptures> load_captures(const std::filesystem::path& dir, const BoardSpec& board = {});

}  // namespace einstar::calibrate
