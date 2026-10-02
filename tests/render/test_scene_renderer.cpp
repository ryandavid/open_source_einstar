#include <catch2/catch_test_macros.hpp>

#include <numbers>
#include <vector>

#include "einstar/gpu/context.hpp"
#include "einstar/render/scene_renderer.hpp"

using namespace einstar;

namespace {

constexpr int W = 256, H = 256;

// Renders the scene into a W x H BGRA image and reads it back.
std::vector<std::uint8_t> render_offscreen(gpu::Context& ctx, render::SceneRenderer& renderer, const render::ViewCamera& cam) {
    auto* dev = ctx.device();
    auto* cd = MTL::TextureDescriptor::texture2DDescriptor(MTL::PixelFormatBGRA8Unorm, W, H, false);
    cd->setUsage(MTL::TextureUsageRenderTarget);
    // Shared textures need unified memory; a discrete GPU reads back through a managed copy.
    const bool unified = dev->hasUnifiedMemory();
    cd->setStorageMode(unified ? MTL::StorageModeShared : MTL::StorageModeManaged);
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

    auto* cmd = ctx.queue()->commandBuffer();
    auto* enc = cmd->renderCommandEncoder(pass);
    renderer.encode(enc, cam, W, H, {});
    enc->endEncoding();
    if (!unified) {
        auto* blit = cmd->blitCommandEncoder();
        blit->synchronizeResource(color.get());
        blit->endEncoding();
    }
    cmd->commit();
    cmd->waitUntilCompleted();

    std::vector<std::uint8_t> px(W * H * 4);
    color->getBytes(px.data(), W * 4, MTL::Region(0, 0, W, H), 0);
    return px;
}

// A 40x40 mm patch of red points at z=300 whose normals face -z (towards the default camera).
std::vector<render::PointVertex> red_patch() {
    std::vector<render::PointVertex> pts;
    for (int y = -20; y <= 20; ++y)
        for (int x = -20; x <= 20; ++x)
            pts.push_back({float(x), float(y), 300.0f, 0, 0, -1, {255, 0, 0, 255}});
    return pts;
}

}  // namespace

TEST_CASE("scene renderer draws splats, markers and lines offscreen") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    auto renderer = render::SceneRenderer::create(*ctx, MTL::PixelFormatBGRA8Unorm, MTL::PixelFormatDepth32Float);
    if (!renderer) FAIL(renderer.error().message);

    // The patch facing the camera, plus a marker and a line.
    (*renderer)->set_model_points(red_patch());
    const render::MarkerInstance marker{60, 0, 300, 0, 0, -1, 3.0f, render::marker_color(render::MarkerState::in_frame)};
    (*renderer)->set_markers(std::span(&marker, 1));
    std::vector<render::LineVertex> lines{{-80, 40, 300, {0, 0, 255, 255}}, {80, 40, 300, {0, 0, 255, 255}}};
    (*renderer)->set_lines(lines);

    render::ViewCamera cam;
    cam.target = {0, 0, 300};
    cam.distance = 300;
    const auto px = render_offscreen(**ctx, **renderer, cam);
    auto at = [&](int x, int y) { return &px[static_cast<std::size_t>(y * W + x) * 4]; };  // BGRA
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

TEST_CASE("points seen from behind take the back colour") {
    auto ctx = gpu::Context::create();
    REQUIRE(ctx.has_value());
    auto renderer = render::SceneRenderer::create(*ctx, MTL::PixelFormatBGRA8Unorm, MTL::PixelFormatDepth32Float);
    if (!renderer) FAIL(renderer.error().message);
    (*renderer)->set_model_points(red_patch());

    render::ViewCamera cam;
    cam.target = {0, 0, 300};
    cam.distance = 300;
    const auto front = render_offscreen(**ctx, **renderer, cam);
    cam.orbit(0.0f, std::numbers::pi_v<float>);  // a half turn (under the target) to the far side: the patch's back
    const auto back = render_offscreen(**ctx, **renderer, cam);
    const auto centre = static_cast<std::size_t>((H / 2) * W + W / 2) * 4;  // BGRA
    // Front: the points' own red. Back: kBackColor (0.55, 0.42, 0.40), face-on so unshaded.
    CHECK(front[centre + 2] > 200);
    CHECK(front[centre + 1] < 40);
    CHECK(back[centre + 2] > 120);
    CHECK(back[centre + 2] < 160);
    CHECK(back[centre + 1] > 90);
    CHECK(back[centre + 1] < 125);
    CHECK(back[centre + 0] > 85);
    CHECK(back[centre + 0] < 120);
}
