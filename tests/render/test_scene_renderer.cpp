#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "einstar/gpu/context.hpp"
#include "einstar/render/scene_renderer.hpp"

using namespace einstar;

TEST_CASE("scene renderer draws splats, markers and lines offscreen") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    auto renderer = render::SceneRenderer::create(*ctx, MTL::PixelFormatBGRA8Unorm, MTL::PixelFormatDepth32Float);
    if (!renderer) FAIL(renderer.error().message);

    // A 40x40 mm patch of points facing the camera at z=300, plus a marker and a line.
    std::vector<render::PointVertex> pts;
    for (int y = -20; y <= 20; ++y)
        for (int x = -20; x <= 20; ++x)
            pts.push_back({float(x), float(y), 300.0f, 0, 0, -1, {255, 0, 0, 255}});
    (*renderer)->set_model_points(pts);
    const render::MarkerInstance marker{60, 0, 300, 0, 0, -1, 3.0f, render::marker_color(render::MarkerState::in_frame)};
    (*renderer)->set_markers(std::span(&marker, 1));
    std::vector<render::LineVertex> lines{{-80, 40, 300, {0, 0, 255, 255}}, {80, 40, 300, {0, 0, 255, 255}}};
    (*renderer)->set_lines(lines);

    constexpr int W = 256, H = 256;
    auto* dev = (*ctx)->device();
    auto* cd = MTL::TextureDescriptor::texture2DDescriptor(MTL::PixelFormatBGRA8Unorm, W, H, false);
    cd->setUsage(MTL::TextureUsageRenderTarget);
    cd->setStorageMode(MTL::StorageModeShared);
    gpu::Ref<MTL::Texture> color(dev->newTexture(cd));
    auto* dd = MTL::TextureDescriptor::texture2DDescriptor(MTL::PixelFormatDepth32Float, W, H, false);
    dd->setUsage(MTL::TextureUsageRenderTarget);
    dd->setStorageMode(MTL::StorageModePrivate);
    gpu::Ref<MTL::Texture> depth(dev->newTexture(dd));

    auto* pass = MTL::RenderPassDescriptor::renderPassDescriptor();
    pass->colorAttachments()->object(0)->setTexture(color.get());
    pass->colorAttachments()->object(0)->setLoadAction(MTL::LoadActionClear);
    pass->colorAttachments()->object(0)->setClearColor(MTL::ClearColor(0, 0, 0, 1));
    pass->colorAttachments()->object(0)->setStoreAction(MTL::StoreActionStore);
    pass->depthAttachment()->setTexture(depth.get());
    pass->depthAttachment()->setLoadAction(MTL::LoadActionClear);
    pass->depthAttachment()->setClearDepth(1.0);

    render::ViewCamera cam;
    cam.target = {0, 0, 300};
    cam.distance = 300;
    auto* cmd = (*ctx)->queue()->commandBuffer();
    auto* enc = cmd->renderCommandEncoder(pass);
    (*renderer)->encode(enc, cam, W, H, {});
    enc->endEncoding();
    cmd->commit();
    cmd->waitUntilCompleted();

    std::vector<std::uint8_t> px(W * H * 4);
    color->getBytes(px.data(), W * 4, MTL::Region(0, 0, W, H), 0);
    auto at = [&](int x, int y) { return &px[(y * W + x) * 4]; };  // BGRA
    // Centre of the image shows the red patch (lit, so red dominates).
    const auto* c = at(W / 2, H / 2);
    CHECK(c[2] > 100);
    CHECK(c[1] < 40);
    // Count coloured pixels by class.
    int red = 0, blue = 0, marker_px = 0;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            const auto* p = at(x, y);
            if (p[2] > 100 && p[1] < 40 && p[0] < 40) ++red;
            if (p[0] > 200 && p[1] < 40 && p[2] < 40) ++blue;
            if (p[0] > 200 && p[1] > 200 && p[2] > 200) ++marker_px;  // white marker centre
        }
    CHECK(red > 1000);
    CHECK(blue > 20);
    CHECK(marker_px > 3);
}
