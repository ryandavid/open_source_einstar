#include "einstar/model/photo_render.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

#include "einstar/image/image.hpp"

namespace einstar::model {
namespace {

constexpr image::Color kInk{255, 214, 10, 255};       // annotations: yellow on a dark edge
constexpr image::Color kEdge{0, 0, 0, 200};
constexpr image::Color kLabel{20, 20, 20, 210};       // label boxes
constexpr image::Color kExtra{0, 220, 255, 230};      // other lines (the model)
constexpr image::Color kGrid{255, 255, 255, 110};

struct Pen {
    image::Canvas& canvas;
    double width;
    void line(const Vec2& a, const Vec2& b, image::Color c) {
        canvas.line(a, b, kEdge, width + 2);
        canvas.line(a, b, c, width);
    }
    void arrow(const Vec2& from, const Vec2& to) {
        canvas.arrow(from, to, kEdge, width + 2);
        canvas.arrow(from, to, kInk, width);
    }
    void circle(const Vec2& c, double r) {
        canvas.circle(c, r, kEdge, width + 2);
        canvas.circle(c, r, kInk, width);
    }
    void dot(const Vec2& c) {
        canvas.dot(c, width + 2.5, kEdge);
        canvas.dot(c, width + 1.5, kInk);
    }
};

}  // namespace

Result<RenderedPhoto> render_photo(const Photo& photo, const PhotoRenderOptions& o) {
    if (!photo.blob) return make_error(Errc::invalid_argument, "the photo has no image");
    std::array<double, 4> crop = {0, 0, static_cast<double>(photo.width), static_cast<double>(photo.height)};
    if (o.crop) {
        const auto& c = *o.crop;
        crop[0] = std::clamp(c[0], 0.0, static_cast<double>(photo.width));
        crop[1] = std::clamp(c[1], 0.0, static_cast<double>(photo.height));
        crop[2] = std::clamp(c[2], 1.0, photo.width - crop[0]);
        crop[3] = std::clamp(c[3], 1.0, photo.height - crop[1]);
    }
    const int max_size = std::clamp(o.max_size, 64, 4096);
    const double scale = std::min(1.0, max_size / std::max(crop[2], crop[3]));
    // Decoded just large enough for the part shown.
    const int full = std::max(photo.width, photo.height);
    const int edge = std::min(full, static_cast<int>(std::ceil(full * scale)) + 1);
    auto decoded = image::decode(photo.blob->bytes, edge);
    if (!decoded) return std::unexpected(decoded.error());
    const double s_dec = static_cast<double>(decoded->width) / photo.width;  // decoded px per photo px
    // Crop and resample to the output size (nearest is enough: the decode already scaled it down).
    RenderedPhoto out;
    out.crop = crop;
    out.width = std::max(1, static_cast<int>(std::lround(crop[2] * scale)));
    out.height = std::max(1, static_cast<int>(std::lround(crop[3] * scale)));
    out.scale = scale;
    image::Rgba img;
    img.width = out.width;
    img.height = out.height;
    img.pixels.resize(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 4);
    for (int y = 0; y < img.height; ++y)
        for (int x = 0; x < img.width; ++x) {
            const int sx = std::clamp(static_cast<int>((crop[0] + (x + 0.5) / scale) * s_dec), 0, decoded->width - 1);
            const int sy = std::clamp(static_cast<int>((crop[1] + (y + 0.5) / scale) * s_dec), 0, decoded->height - 1);
            std::copy_n(&decoded->pixels[(static_cast<std::size_t>(sy) * static_cast<std::size_t>(decoded->width) + static_cast<std::size_t>(sx)) * 4], 4,
                        &img.pixels[(static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4]);
        }

    {
        image::Canvas canvas(img);
        const auto at = [&](const Vec2& p) { return Vec2((p.x() - crop[0]) * scale, (p.y() - crop[1]) * scale); };
        const double text_size = std::clamp(std::max(img.width, img.height) / 55.0, 11.0, 28.0);
        Pen pen{canvas, std::clamp(std::max(img.width, img.height) / 600.0, 1.5, 4.0)};
        const auto label = [&](const Vec2& p, const std::string& s) { canvas.text(p + Vec2(6, 6), s, text_size, kInk, kLabel); };

        if (o.grid) {
            // About ten lines across the longer side, at round photo-pixel steps.
            const double raw = std::max(crop[2], crop[3]) / 10;
            const double mag = std::pow(10.0, std::floor(std::log10(raw)));
            const double step = raw / mag < 2 ? 2 * mag : raw / mag < 5 ? 5 * mag : 10 * mag;
            for (double x = std::ceil(crop[0] / step) * step; x < crop[0] + crop[2]; x += step) {
                canvas.line(at({x, crop[1]}), at({x, crop[1] + crop[3]}), kGrid, 1);
                canvas.text(at({x, crop[1]}) + Vec2(2, 2), std::format("{:.0f}", x), text_size * 0.7, kGrid);
            }
            for (double y = std::ceil(crop[1] / step) * step; y < crop[1] + crop[3]; y += step) {
                canvas.line(at({crop[0], y}), at({crop[0] + crop[2], y}), kGrid, 1);
                canvas.text(at({crop[0], y}) + Vec2(2, 2), std::format("{:.0f}", y), text_size * 0.7, kGrid);
            }
        }
        for (const auto& [a, b] : o.lines) canvas.line(at(a), at(b), kExtra, pen.width * 0.6);

        if (o.annotations)
            for (const auto& a : photo.annotations) {
                std::vector<Vec2> p;
                for (const auto& q : a.points) p.push_back(at(q));
                const std::string text = annotation_summary(a);
                switch (a.kind) {
                    case AnnotationKind::dimension: {
                        if (p.size() < 2) break;
                        pen.line(p[0], p[1], kInk);
                        const Vec2 d = p[1] - p[0];
                        const Vec2 n = d.norm() > 1e-9 ? Vec2(-d.y(), d.x()).normalized() * 3 * pen.width : Vec2(0, 0);
                        pen.line(p[0] - n, p[0] + n, kInk);
                        pen.line(p[1] - n, p[1] + n, kInk);
                        label(0.5 * (p[0] + p[1]), text);
                        break;
                    }
                    case AnnotationKind::diameter: {
                        const auto c = annotation_circle(a);
                        if (c) {
                            pen.circle(at(c->first), c->second * scale);
                            label(at(c->first) + Vec2(c->second * scale * 0.7, -c->second * scale * 0.7), text);
                        }
                        for (const auto& q : p) pen.dot(q);
                        break;
                    }
                    case AnnotationKind::angle: {
                        if (p.size() < 3) break;
                        pen.line(p[1], p[0], kInk);
                        pen.line(p[1], p[2], kInk);
                        label(p[1], text);
                        break;
                    }
                    case AnnotationKind::callout: {
                        if (p.size() < 2) break;
                        pen.arrow(p[1], p[0]);
                        label(p[1], text);
                        break;
                    }
                    case AnnotationKind::note: {
                        if (p.empty()) break;
                        pen.dot(p[0]);
                        label(p[0], text);
                        break;
                    }
                }
            }
        canvas.finish();
    }
    out.jpeg = image::encode_jpeg(img, 0.85);
    if (out.jpeg.empty()) return make_error(Errc::io, "the photo could not be encoded");
    return out;
}

}  // namespace einstar::model
