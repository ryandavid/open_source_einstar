#pragma once

// A rendered frame for ui.screenshot, and turning (a part of) it into a PNG.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace einstar::agent {

struct Frame {
    int width = 0, height = 0;          // framebuffer px
    float scale = 1;                    // framebuffer px per window point
    std::vector<std::uint8_t> bgra;     // width * height * 4, rows top to bottom
};

struct Png {
    std::string bytes;        // the PNG file
    int width = 0, height = 0;
    float scale = 1;          // image px per window point
    float rect[4] = {};       // the logical rect [x, y, w, h] in points
};

// The rect [x, y, w, h] in points (clamped to the frame; the whole frame if absent), downscaled so its
// longer side is at most max_size px. nullopt if the rect is empty or encoding fails.
[[nodiscard]] std::optional<Png> encode_png(const Frame& frame, const float* rect, int max_size);

[[nodiscard]] std::string base64(std::string_view bytes);

}  // namespace einstar::agent
