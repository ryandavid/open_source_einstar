#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
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

// Page-aligned storage (16 KiB, the Apple Silicon page), rounded up to whole pages: the GPU can use an
// image's memory in place (a no-copy Metal buffer) instead of copying it, e.g. raw camera frames.
inline constexpr std::size_t kImagePageBytes = 16384;

template <typename T>
struct PageAlignedAllocator {
    using value_type = T;
    PageAlignedAllocator() = default;
    template <typename U>
    PageAlignedAllocator(const PageAlignedAllocator<U>&) noexcept {}
    [[nodiscard]] T* allocate(std::size_t n) {
        const std::size_t bytes = (n * sizeof(T) + kImagePageBytes - 1) / kImagePageBytes * kImagePageBytes;
        void* p = std::aligned_alloc(kImagePageBytes, std::max(bytes, kImagePageBytes));
        if (!p) throw std::bad_alloc();
        return static_cast<T*>(p);
    }
    void deallocate(T* p, std::size_t) noexcept { std::free(p); }
    template <typename U>
    bool operator==(const PageAlignedAllocator<U>&) const noexcept {
        return true;
    }
};

// Owning, tightly packed 2D image.
template <typename T>
class Image {
public:
    Image() = default;
    Image(int w, int h, T fill = T{}) : width_(w), height_(h), pixels_(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), fill) {}

    [[nodiscard]] int width() const { return width_; }
    [[nodiscard]] int height() const { return height_; }
    [[nodiscard]] std::size_t size() const { return pixels_.size(); }
    [[nodiscard]] bool empty() const { return pixels_.empty(); }

    [[nodiscard]] T* data() { return pixels_.data(); }
    [[nodiscard]] const T* data() const { return pixels_.data(); }
    [[nodiscard]] std::span<T> pixels() { return pixels_; }
    [[nodiscard]] std::span<const T> pixels() const { return pixels_; }

    [[nodiscard]] T& operator()(int x, int y) { return pixels_[index(x, y)]; }
    [[nodiscard]] const T& operator()(int x, int y) const { return pixels_[index(x, y)]; }

    [[nodiscard]] ImageView<T> view() { return {pixels_.data(), width_, height_}; }
    [[nodiscard]] ImageView<const T> view() const { return {pixels_.data(), width_, height_}; }

    void fill(T v) { std::fill(pixels_.begin(), pixels_.end(), v); }

private:
    [[nodiscard]] std::size_t index(int x, int y) const {
        assert(x >= 0 && x < width_ && y >= 0 && y < height_);
        return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
    }

    int width_ = 0;
    int height_ = 0;
    std::vector<T, PageAlignedAllocator<T>> pixels_;
};

using ImageU8 = Image<std::uint8_t>;
using ImageU16 = Image<std::uint16_t>;
using ImageF32 = Image<float>;

}  // namespace einstar
