#include "einstar/agent/image.hpp"

#include <algorithm>
#include <cmath>

#include <CoreGraphics/CoreGraphics.h>
#include <ImageIO/ImageIO.h>

namespace einstar::agent {

std::optional<Png> encode_png(const Frame& f, const float* rect, int max_size) {
    if (f.width <= 0 || f.height <= 0 || f.bgra.size() < static_cast<std::size_t>(f.width) * static_cast<std::size_t>(f.height) * 4)
        return std::nullopt;
    // The rect in framebuffer px, clamped.
    const float pts_w = static_cast<float>(f.width) / f.scale, pts_h = static_cast<float>(f.height) / f.scale;
    float r[4] = {0, 0, pts_w, pts_h};
    if (rect) {
        r[0] = std::clamp(rect[0], 0.0f, pts_w);
        r[1] = std::clamp(rect[1], 0.0f, pts_h);
        r[2] = std::clamp(rect[0] + rect[2], 0.0f, pts_w) - r[0];
        r[3] = std::clamp(rect[1] + rect[3], 0.0f, pts_h) - r[1];
    }
    const auto px = static_cast<std::size_t>(std::lround(r[0] * f.scale)), py = static_cast<std::size_t>(std::lround(r[1] * f.scale));
    const auto pw = static_cast<std::size_t>(std::lround(r[2] * f.scale)), ph = static_cast<std::size_t>(std::lround(r[3] * f.scale));
    if (pw == 0 || ph == 0) return std::nullopt;
    const double shrink = max_size > 0 ? std::min(1.0, static_cast<double>(max_size) / static_cast<double>(std::max(pw, ph))) : 1.0;
    const auto ow = std::max<std::size_t>(1, static_cast<std::size_t>(std::lround(static_cast<double>(pw) * shrink)));
    const auto oh = std::max<std::size_t>(1, static_cast<std::size_t>(std::lround(static_cast<double>(ph) * shrink)));

    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
    const auto info = static_cast<CGBitmapInfo>(kCGImageAlphaPremultipliedFirst) | static_cast<CGBitmapInfo>(kCGBitmapByteOrder32Little);
    CGContextRef src = CGBitmapContextCreate(const_cast<std::uint8_t*>(f.bgra.data()), static_cast<std::size_t>(f.width),
                                             static_cast<std::size_t>(f.height), 8, static_cast<std::size_t>(f.width) * 4, cs, info);
    CGImageRef whole = src ? CGBitmapContextCreateImage(src) : nullptr;
    CGImageRef crop = whole ? CGImageCreateWithImageInRect(whole, CGRectMake(static_cast<CGFloat>(px), static_cast<CGFloat>(py),
                                                                             static_cast<CGFloat>(pw), static_cast<CGFloat>(ph)))
                            : nullptr;
    CGImageRef out = nullptr;
    if (crop && (ow != pw || oh != ph)) {
        CGContextRef dst = CGBitmapContextCreate(nullptr, ow, oh, 8, 0, cs, info);
        if (dst) {
            CGContextSetInterpolationQuality(dst, kCGInterpolationHigh);
            CGContextDrawImage(dst, CGRectMake(0, 0, static_cast<CGFloat>(ow), static_cast<CGFloat>(oh)), crop);
            out = CGBitmapContextCreateImage(dst);
            CGContextRelease(dst);
        }
    } else if (crop) {
        out = CGImageRetain(crop);
    }
    std::optional<Png> png;
    if (out) {
        CFMutableDataRef data = CFDataCreateMutable(nullptr, 0);
        CGImageDestinationRef dest = CGImageDestinationCreateWithData(data, CFSTR("public.png"), 1, nullptr);
        if (dest) {
            CGImageDestinationAddImage(dest, out, nullptr);
            if (CGImageDestinationFinalize(dest)) {
                Png p;
                p.bytes.assign(reinterpret_cast<const char*>(CFDataGetBytePtr(data)), static_cast<std::size_t>(CFDataGetLength(data)));
                p.width = static_cast<int>(ow);
                p.height = static_cast<int>(oh);
                p.scale = f.scale * static_cast<float>(shrink);
                std::copy_n(r, 4, p.rect);
                png = std::move(p);
            }
            CFRelease(dest);
        }
        CFRelease(data);
        CGImageRelease(out);
    }
    if (crop) CGImageRelease(crop);
    if (whole) CGImageRelease(whole);
    if (src) CGContextRelease(src);
    CGColorSpaceRelease(cs);
    return png;
}

std::string base64(std::string_view in) {
    static constexpr char kTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((in.size() + 2) / 3 * 4);
    std::size_t i = 0;
    for (; i + 2 < in.size(); i += 3) {
        const std::uint32_t v = (static_cast<std::uint32_t>(static_cast<std::uint8_t>(in[i])) << 16) |
                                (static_cast<std::uint32_t>(static_cast<std::uint8_t>(in[i + 1])) << 8) | static_cast<std::uint8_t>(in[i + 2]);
        for (const int s : {18, 12, 6, 0}) out.push_back(kTable[(v >> s) & 63]);
    }
    if (i < in.size()) {
        std::uint32_t v = static_cast<std::uint32_t>(static_cast<std::uint8_t>(in[i])) << 16;
        if (i + 1 < in.size()) v |= static_cast<std::uint32_t>(static_cast<std::uint8_t>(in[i + 1])) << 8;
        out.push_back(kTable[(v >> 18) & 63]);
        out.push_back(kTable[(v >> 12) & 63]);
        out.push_back(i + 1 < in.size() ? kTable[(v >> 6) & 63] : '=');
        out.push_back('=');
    }
    return out;
}

}  // namespace einstar::agent
