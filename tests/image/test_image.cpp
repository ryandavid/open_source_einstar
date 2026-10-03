#include <catch2/catch_test_macros.hpp>

#include "einstar/image/image.hpp"

using namespace einstar;

namespace {

// 40 x 20: the left half red, the right half blue.
image::Rgba halves() {
    image::Rgba img;
    img.width = 40;
    img.height = 20;
    img.pixels.resize(40 * 20 * 4);
    for (int y = 0; y < 20; ++y)
        for (int x = 0; x < 40; ++x) {
            auto* p = &img.pixels[static_cast<std::size_t>((y * 40 + x) * 4)];
            p[0] = x < 20 ? 255 : 0;
            p[2] = x < 20 ? 0 : 255;
            p[3] = 255;
        }
    return img;
}

bool reddish(const image::Rgba& img, int x, int y) {
    const auto* p = &img.pixels[static_cast<std::size_t>((y * img.width + x) * 4)];
    return p[0] > 200 && p[2] < 60;
}
bool bluish(const image::Rgba& img, int x, int y) {
    const auto* p = &img.pixels[static_cast<std::size_t>((y * img.width + x) * 4)];
    return p[2] > 200 && p[0] < 60;
}

}  // namespace

TEST_CASE("photos decode as shown: a phone photo stored on its side comes out upright") {
    const std::string plain = image::encode_jpeg(halves());
    const auto info = image::probe(plain);
    REQUIRE(info);
    CHECK(info->mime == "image/jpeg");
    CHECK((info->width == 40 && info->height == 20));

    // Orientation 6: the viewer turns the stored pixels 90 degrees clockwise, so the stored left (red) is the top.
    const std::string turned = image::encode_jpeg(halves(), 0.9, 6);
    const auto tinfo = image::probe(turned);
    REQUIRE(tinfo);
    CHECK(tinfo->orientation == 6);
    CHECK((tinfo->width == 20 && tinfo->height == 40));
    const auto img = image::decode(turned);
    REQUIRE(img);
    REQUIRE((img->width == 20 && img->height == 40));
    CHECK(reddish(*img, 10, 5));
    CHECK(bluish(*img, 10, 35));

    const auto small = image::decode(turned, 10);
    REQUIRE(small);
    CHECK(std::max(small->width, small->height) == 10);

    CHECK(!image::probe("not an image"));
    CHECK(image::content_hash(plain) == image::content_hash(plain));
    CHECK(image::content_hash(plain) != image::content_hash(turned));
}

TEST_CASE("drawing over a photo lands where its pixel coordinates say") {
    image::Rgba img = halves();
    {
        image::Canvas c(img);
        c.dot({5, 5}, 3, {0, 255, 0, 255});  // top left, in image coordinates (y down)
        c.finish();
    }
    const auto* p = &img.pixels[static_cast<std::size_t>((5 * 40 + 5) * 4)];
    CHECK((p[1] > 200 && p[0] < 60));
    const auto* q = &img.pixels[static_cast<std::size_t>((15 * 40 + 5) * 4)];
    CHECK(q[0] > 200);  // the bottom left is still red
    CHECK(!image::encode_png(img).empty());
}
