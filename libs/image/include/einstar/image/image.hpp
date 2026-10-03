#pragma once

// Photos: what they are (size as shown, type, the camera's EXIF), their pixels, and drawing over them.
//
// Sizes and pixel coordinates are always of the image as it is shown: EXIF orientation applied (a phone photo
// taken upright is taller than wide), origin top left, x right, y down.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "einstar/core/error.hpp"
#include "einstar/core/se3.hpp"

namespace einstar::image {

struct Exif {
    std::string make, model;   // the camera
    std::string taken;         // "YYYY:MM:DD HH:MM:SS" as recorded
    std::optional<double> focal_mm;       // the lens's focal length
    std::optional<double> focal_35mm;     // its 35 mm equivalent (for the field of view)
};

struct Info {
    int width = 0, height = 0;  // as shown
    std::string mime;           // "image/jpeg", "image/heic", "image/png", ...
    int orientation = 1;        // EXIF orientation of the stored pixels (1: as shown)
    Exif exif;
};

// Reads the header only. An error if the bytes are not an image ImageIO knows.
[[nodiscard]] Result<Info> probe(std::string_view bytes);

// 8-bit RGBA, not premultiplied, rows top to bottom.
struct Rgba {
    int width = 0, height = 0;
    std::vector<std::uint8_t> pixels;
};

// The image as shown, its longer side scaled down to max_edge if it is longer (0: full size).
[[nodiscard]] Result<Rgba> decode(std::string_view bytes, int max_edge = 0);

// `orientation` (EXIF, 1: as is) is recorded in the file: how a viewer should turn the pixels to show them.
[[nodiscard]] std::string encode_jpeg(const Rgba& image, double quality = 0.85, int orientation = 1);
[[nodiscard]] std::string encode_png(const Rgba& image);

// A content hash of a file's bytes (64-bit FNV-1a, hex): blobs are stored once however often they are used.
[[nodiscard]] std::string content_hash(std::string_view bytes);

struct Color {
    std::uint8_t r = 255, g = 255, b = 255, a = 255;
};

// Draws over an image, in its pixel coordinates.
class Canvas {
public:
    explicit Canvas(Rgba& image);
    ~Canvas();
    Canvas(const Canvas&) = delete;
    Canvas& operator=(const Canvas&) = delete;

    void line(const Vec2& a, const Vec2& b, Color c, double width);
    void circle(const Vec2& center, double radius, Color c, double width);
    void dot(const Vec2& center, double radius, Color c);
    // A line from `from` with an arrowhead at `to`.
    void arrow(const Vec2& from, const Vec2& to, Color c, double width);
    // Text with its top left at `at`, on a box of `background` (alpha 0: none). Returns its size.
    Vec2 text(const Vec2& at, std::string_view s, double size, Color c, Color background = {0, 0, 0, 0});
    // Writes the drawing back into the image.
    void finish();

private:
    Rgba& image_;
    void* context_ = nullptr;  // CGContextRef
};

}  // namespace einstar::image
