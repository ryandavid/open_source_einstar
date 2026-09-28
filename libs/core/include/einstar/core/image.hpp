#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <vector>

namespace einstar {

// Non-owning 2D view. Stride is in elements, not bytes.
template <typename T>
struct ImageView {
    T* data = nullptr;
    int width = 0;
    int height = 0;
    std::ptrdiff_t stride = 0;

    constexpr ImageView() = default;
    constexpr ImageView(T* d, int w, int h, std::ptrdiff_t s) : data(d), width(w), height(h), stride(s) {}
    constexpr ImageView(T* d, int w, int h) : ImageView(d, w, h, w) {}

    // Allow ImageView<T> -> ImageView<const T>.
    template <typename U>
        requires std::is_same_v<const U, T>
    constexpr ImageView(const ImageView<U>& o) : data(o.data), width(o.width), height(o.height), stride(o.stride) {}

    [[nodiscard]] constexpr T& operator()(int x, int y) const {
        assert(x >= 0 && x < width && y >= 0 && y < height);
        return data[y * stride + x];
    }
    [[nodiscard]] constexpr T* row(int y) const { return data + y * stride; }
    [[nodiscard]] constexpr bool empty() const { return data == nullptr || width == 0 || height == 0; }
    [[nodiscard]] constexpr bool contains(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
};

// Owning, tightly packed 2D image.
template <typename T>
class Image {
public:
    Image() = default;
    Image(int w, int h, T fill = T{}) : width_(w), height_(h), pixels_(static_cast<std::size_t>(w) * h, fill) {}

    [[nodiscard]] int width() const { return width_; }
    [[nodiscard]] int height() const { return height_; }
    [[nodiscard]] std::size_t size() const { return pixels_.size(); }
    [[nodiscard]] bool empty() const { return pixels_.empty(); }

    [[nodiscard]] T* data() { return pixels_.data(); }
    [[nodiscard]] const T* data() const { return pixels_.data(); }
    [[nodiscard]] std::span<T> pixels() { return pixels_; }
    [[nodiscard]] std::span<const T> pixels() const { return pixels_; }

    [[nodiscard]] T& operator()(int x, int y) { return pixels_[static_cast<std::size_t>(y) * width_ + x]; }
    [[nodiscard]] const T& operator()(int x, int y) const { return pixels_[static_cast<std::size_t>(y) * width_ + x]; }

    [[nodiscard]] ImageView<T> view() { return {pixels_.data(), width_, height_}; }
    [[nodiscard]] ImageView<const T> view() const { return {pixels_.data(), width_, height_}; }

    void fill(T v) { std::fill(pixels_.begin(), pixels_.end(), v); }

private:
    int width_ = 0;
    int height_ = 0;
    std::vector<T> pixels_;
};

using ImageU8 = Image<std::uint8_t>;
using ImageU16 = Image<std::uint16_t>;
using ImageF32 = Image<float>;

}  // namespace einstar
