#include "einstar/image/image.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

#include <CoreFoundation/CoreFoundation.h>
#include <CoreGraphics/CoreGraphics.h>
#include <CoreText/CoreText.h>
#include <ImageIO/ImageIO.h>

namespace einstar::image {
namespace {

// Releases a CoreFoundation object at the end of a scope.
template <class T>
struct Cf {
    T ref = nullptr;
    Cf() = default;
    explicit Cf(T r) : ref(r) {}
    ~Cf() {
        if (ref) CFRelease(ref);
    }
    Cf(const Cf&) = delete;
    Cf& operator=(const Cf&) = delete;
    explicit operator bool() const { return ref != nullptr; }
};

std::string to_string(CFStringRef s) {
    if (!s) return {};
    const CFIndex n = CFStringGetMaximumSizeForEncoding(CFStringGetLength(s), kCFStringEncodingUTF8) + 1;
    std::string out(static_cast<std::size_t>(n), '\0');
    if (!CFStringGetCString(s, out.data(), n, kCFStringEncodingUTF8)) return {};
    out.resize(std::char_traits<char>::length(out.c_str()));
    return out;
}

std::string string_in(CFDictionaryRef dict, CFStringRef key) {
    if (!dict) return {};
    const auto v = static_cast<CFTypeRef>(CFDictionaryGetValue(dict, key));
    return v && CFGetTypeID(v) == CFStringGetTypeID() ? to_string(static_cast<CFStringRef>(v)) : std::string();
}

std::optional<double> number_in(CFDictionaryRef dict, CFStringRef key) {
    if (!dict) return std::nullopt;
    const auto v = static_cast<CFTypeRef>(CFDictionaryGetValue(dict, key));
    double d = 0;
    if (!v || CFGetTypeID(v) != CFNumberGetTypeID() || !CFNumberGetValue(static_cast<CFNumberRef>(v), kCFNumberDoubleType, &d)) return std::nullopt;
    return d;
}

CFDictionaryRef dict_in(CFDictionaryRef dict, CFStringRef key) {
    if (!dict) return nullptr;
    const auto v = static_cast<CFTypeRef>(CFDictionaryGetValue(dict, key));
    return v && CFGetTypeID(v) == CFDictionaryGetTypeID() ? static_cast<CFDictionaryRef>(v) : nullptr;
}

Cf<CGImageSourceRef> source_of(std::string_view bytes) {
    Cf<CFDataRef> data(CFDataCreate(nullptr, reinterpret_cast<const UInt8*>(bytes.data()), static_cast<CFIndex>(bytes.size())));
    if (!data) return Cf<CGImageSourceRef>();
    return Cf<CGImageSourceRef>(CGImageSourceCreateWithData(data.ref, nullptr));
}

std::string mime_of(const std::string& uti) {
    if (uti == "public.jpeg") return "image/jpeg";
    if (uti == "public.png") return "image/png";
    if (uti == "public.heic") return "image/heic";
    if (uti == "public.heif") return "image/heif";
    if (uti == "public.tiff") return "image/tiff";
    if (uti == "org.webmproject.webp") return "image/webp";
    return "image/" + uti;
}

constexpr auto kRgbaInfo = static_cast<CGBitmapInfo>(kCGImageAlphaPremultipliedLast) | static_cast<CGBitmapInfo>(kCGBitmapByteOrder32Big);

void premultiply(std::vector<std::uint8_t>& p) {
    for (std::size_t i = 0; i + 3 < p.size(); i += 4)
        if (const unsigned a = p[i + 3]; a != 255)
            for (std::size_t c = 0; c < 3; ++c) p[i + c] = static_cast<std::uint8_t>((p[i + c] * a + 127) / 255);
}

void unpremultiply(std::vector<std::uint8_t>& p) {
    for (std::size_t i = 0; i + 3 < p.size(); i += 4)
        if (const unsigned a = p[i + 3]; a != 255 && a != 0)
            for (std::size_t c = 0; c < 3; ++c) p[i + c] = static_cast<std::uint8_t>(std::min(255u, (p[i + c] * 255 + a / 2) / a));
}

// An image's pixels as a CGImage (copied).
Cf<CGImageRef> cg_image(const Rgba& image) {
    auto pixels = image.pixels;
    premultiply(pixels);
    Cf<CGColorSpaceRef> cs(CGColorSpaceCreateWithName(kCGColorSpaceSRGB));
    Cf<CFDataRef> data(CFDataCreate(nullptr, pixels.data(), static_cast<CFIndex>(pixels.size())));
    Cf<CGDataProviderRef> provider(CGDataProviderCreateWithCFData(data.ref));
    return Cf<CGImageRef>(CGImageCreate(static_cast<std::size_t>(image.width), static_cast<std::size_t>(image.height), 8, 32,
                                        static_cast<std::size_t>(image.width) * 4, cs.ref, kRgbaInfo, provider.ref, nullptr, false,
                                        kCGRenderingIntentDefault));
}

std::string encode(const Rgba& image, CFStringRef type, std::optional<double> quality, int orientation = 1) {
    if (image.width <= 0 || image.height <= 0) return {};
    Cf<CGImageRef> img = cg_image(image);
    if (!img) return {};
    Cf<CFMutableDataRef> data(CFDataCreateMutable(nullptr, 0));
    Cf<CGImageDestinationRef> dest(CGImageDestinationCreateWithData(data.ref, type, 1, nullptr));
    if (!dest) return {};
    Cf<CFMutableDictionaryRef> props(CFDictionaryCreateMutable(nullptr, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks));
    if (quality) {
        Cf<CFNumberRef> q(CFNumberCreate(nullptr, kCFNumberDoubleType, &*quality));
        CFDictionarySetValue(props.ref, kCGImageDestinationLossyCompressionQuality, q.ref);
    }
    if (orientation != 1) {
        Cf<CFNumberRef> o(CFNumberCreate(nullptr, kCFNumberIntType, &orientation));
        CFDictionarySetValue(props.ref, kCGImagePropertyOrientation, o.ref);
    }
    CGImageDestinationAddImage(dest.ref, img.ref, props.ref);
    if (!CGImageDestinationFinalize(dest.ref)) return {};
    return std::string(reinterpret_cast<const char*>(CFDataGetBytePtr(data.ref)), static_cast<std::size_t>(CFDataGetLength(data.ref)));
}

CGColorRef cg_color(Color c) {
    return CGColorCreateSRGB(c.r / 255.0, c.g / 255.0, c.b / 255.0, c.a / 255.0);
}

}  // namespace

Result<Info> probe(std::string_view bytes) {
    auto src = source_of(bytes);
    if (!src || CGImageSourceGetCount(src.ref) < 1) return make_error(Errc::invalid_argument, "not an image (or a kind this Mac cannot read)");
    Cf<CFDictionaryRef> props(CGImageSourceCopyPropertiesAtIndex(src.ref, 0, nullptr));
    if (!props) return make_error(Errc::invalid_argument, "an image without properties");
    Info info;
    info.mime = mime_of(to_string(CGImageSourceGetType(src.ref)));
    const int w = static_cast<int>(number_in(props.ref, kCGImagePropertyPixelWidth).value_or(0));
    const int h = static_cast<int>(number_in(props.ref, kCGImagePropertyPixelHeight).value_or(0));
    if (w <= 0 || h <= 0) return make_error(Errc::invalid_argument, "an image without a size");
    info.orientation = static_cast<int>(number_in(props.ref, kCGImagePropertyOrientation).value_or(1));
    const bool turned = info.orientation >= 5 && info.orientation <= 8;  // stored on its side
    info.width = turned ? h : w;
    info.height = turned ? w : h;
    const CFDictionaryRef exif = dict_in(props.ref, kCGImagePropertyExifDictionary);
    const CFDictionaryRef tiff = dict_in(props.ref, kCGImagePropertyTIFFDictionary);
    info.exif.make = string_in(tiff, kCGImagePropertyTIFFMake);
    info.exif.model = string_in(tiff, kCGImagePropertyTIFFModel);
    info.exif.taken = string_in(exif, kCGImagePropertyExifDateTimeOriginal);
    info.exif.focal_mm = number_in(exif, kCGImagePropertyExifFocalLength);
    info.exif.focal_35mm = number_in(exif, kCGImagePropertyExifFocalLenIn35mmFilm);
    if (info.exif.focal_35mm && *info.exif.focal_35mm <= 0) info.exif.focal_35mm.reset();
    return info;
}

Result<Rgba> decode(std::string_view bytes, int max_edge) {
    const auto info = probe(bytes);
    if (!info) return std::unexpected(info.error());
    auto src = source_of(bytes);
    const int full = std::max(info->width, info->height);
    int edge = max_edge > 0 ? std::min(max_edge, full) : full;
    Cf<CFNumberRef> size(CFNumberCreate(nullptr, kCFNumberIntType, &edge));
    const void* keys[] = {kCGImageSourceCreateThumbnailFromImageAlways, kCGImageSourceCreateThumbnailWithTransform, kCGImageSourceThumbnailMaxPixelSize,
                          kCGImageSourceShouldCacheImmediately};
    const void* values[] = {kCFBooleanTrue, kCFBooleanTrue, size.ref, kCFBooleanTrue};
    Cf<CFDictionaryRef> opts(CFDictionaryCreate(nullptr, keys, values, 4, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks));
    Cf<CGImageRef> img(CGImageSourceCreateThumbnailAtIndex(src.ref, 0, opts.ref));
    if (!img) return make_error(Errc::invalid_argument, "the image could not be decoded");
    Rgba out;
    out.width = static_cast<int>(CGImageGetWidth(img.ref));
    out.height = static_cast<int>(CGImageGetHeight(img.ref));
    out.pixels.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 4, 0);
    Cf<CGColorSpaceRef> cs(CGColorSpaceCreateWithName(kCGColorSpaceSRGB));
    Cf<CGContextRef> ctx(CGBitmapContextCreate(out.pixels.data(), static_cast<std::size_t>(out.width), static_cast<std::size_t>(out.height), 8,
                                               static_cast<std::size_t>(out.width) * 4, cs.ref, kRgbaInfo));
    if (!ctx) return make_error(Errc::invalid_argument, "the image could not be drawn");
    CGContextDrawImage(ctx.ref, CGRectMake(0, 0, out.width, out.height), img.ref);
    unpremultiply(out.pixels);
    return out;
}

std::string encode_jpeg(const Rgba& image, double quality, int orientation) { return encode(image, CFSTR("public.jpeg"), quality, orientation); }
std::string encode_png(const Rgba& image) { return encode(image, CFSTR("public.png"), std::nullopt); }

std::string content_hash(std::string_view bytes) {
    std::uint64_t h = 14695981039346656037ull;
    for (const char c : bytes) {
        h ^= static_cast<std::uint8_t>(c);
        h *= 1099511628211ull;
    }
    return std::format("{:016x}", h);
}

// ---- Canvas ----

Canvas::Canvas(Rgba& image) : image_(image) {
    premultiply(image_.pixels);
    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef ctx = CGBitmapContextCreate(image_.pixels.data(), static_cast<std::size_t>(image_.width), static_cast<std::size_t>(image_.height), 8,
                                             static_cast<std::size_t>(image_.width) * 4, cs, kRgbaInfo);
    CGColorSpaceRelease(cs);
    if (ctx) {
        // Image coordinates: origin top left, y down.
        CGContextTranslateCTM(ctx, 0, image_.height);
        CGContextScaleCTM(ctx, 1, -1);
        CGContextSetLineCap(ctx, kCGLineCapRound);
        CGContextSetLineJoin(ctx, kCGLineJoinRound);
        CGContextSetShouldAntialias(ctx, true);
    }
    context_ = ctx;
}

Canvas::~Canvas() { finish(); }

void Canvas::finish() {
    if (!context_) return;
    CGContextFlush(static_cast<CGContextRef>(context_));
    CGContextRelease(static_cast<CGContextRef>(context_));
    context_ = nullptr;
    unpremultiply(image_.pixels);
}

void Canvas::line(const Vec2& a, const Vec2& b, Color c, double width) {
    auto ctx = static_cast<CGContextRef>(context_);
    if (!ctx) return;
    Cf<CGColorRef> col(cg_color(c));
    CGContextSetStrokeColorWithColor(ctx, col.ref);
    CGContextSetLineWidth(ctx, width);
    CGContextBeginPath(ctx);
    CGContextMoveToPoint(ctx, a.x(), a.y());
    CGContextAddLineToPoint(ctx, b.x(), b.y());
    CGContextStrokePath(ctx);
}

void Canvas::circle(const Vec2& center, double radius, Color c, double width) {
    auto ctx = static_cast<CGContextRef>(context_);
    if (!ctx) return;
    Cf<CGColorRef> col(cg_color(c));
    CGContextSetStrokeColorWithColor(ctx, col.ref);
    CGContextSetLineWidth(ctx, width);
    CGContextStrokeEllipseInRect(ctx, CGRectMake(center.x() - radius, center.y() - radius, 2 * radius, 2 * radius));
}

void Canvas::dot(const Vec2& center, double radius, Color c) {
    auto ctx = static_cast<CGContextRef>(context_);
    if (!ctx) return;
    Cf<CGColorRef> col(cg_color(c));
    CGContextSetFillColorWithColor(ctx, col.ref);
    CGContextFillEllipseInRect(ctx, CGRectMake(center.x() - radius, center.y() - radius, 2 * radius, 2 * radius));
}

void Canvas::arrow(const Vec2& from, const Vec2& to, Color c, double width) {
    line(from, to, c, width);
    const Vec2 d = to - from;
    if (d.norm() < 1e-9) return;
    const Vec2 u = d.normalized();
    const Vec2 n(-u.y(), u.x());
    const double head = std::max(4 * width, 8.0);
    line(to, to - head * u + 0.5 * head * n, c, width);
    line(to, to - head * u - 0.5 * head * n, c, width);
}

Vec2 Canvas::text(const Vec2& at, std::string_view s, double size, Color c, Color background) {
    auto ctx = static_cast<CGContextRef>(context_);
    if (!ctx || s.empty()) return Vec2::Zero();
    Cf<CFStringRef> str(CFStringCreateWithBytes(nullptr, reinterpret_cast<const UInt8*>(s.data()), static_cast<CFIndex>(s.size()), kCFStringEncodingUTF8, false));
    if (!str) return Vec2::Zero();
    Cf<CTFontRef> font(CTFontCreateUIFontForLanguage(kCTFontUIFontEmphasizedSystem, size, nullptr));
    Cf<CGColorRef> col(cg_color(c));
    const void* keys[] = {kCTFontAttributeName, kCTForegroundColorAttributeName};
    const void* values[] = {font.ref, col.ref};
    Cf<CFDictionaryRef> attrs(CFDictionaryCreate(nullptr, keys, values, 2, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks));
    Cf<CFAttributedStringRef> astr(CFAttributedStringCreate(nullptr, str.ref, attrs.ref));
    Cf<CTLineRef> line_ref(CTLineCreateWithAttributedString(astr.ref));
    CGFloat ascent = 0, descent = 0, leading = 0;
    const double w = CTLineGetTypographicBounds(line_ref.ref, &ascent, &descent, &leading);
    const double h = ascent + descent;
    const double pad = 0.25 * size;
    if (background.a > 0) {
        Cf<CGColorRef> bg(cg_color(background));
        CGContextSetFillColorWithColor(ctx, bg.ref);
        CGContextFillRect(ctx, CGRectMake(at.x(), at.y(), w + 2 * pad, h + 2 * pad));
    }
    // The context is flipped (y down): flip the glyphs back.
    CGContextSetTextMatrix(ctx, CGAffineTransformMakeScale(1, -1));
    CGContextSetTextPosition(ctx, at.x() + pad, at.y() + pad + ascent);
    CTLineDraw(line_ref.ref, ctx);
    return {w + 2 * pad, h + 2 * pad};
}

}  // namespace einstar::image
