#pragma once

// A camera image drawn upright, the way the operator sees it with the scanner held upright (its top, the
// right camera, up; the EINSTAR logo reading). The cameras sit along the scanner, so the image's +x runs up
// the scanner: drawn turned 90 degrees anticlockwise, image +x is up the screen and image +y to the right
// ("the view", calibrate/plan.hpp). Rectified images keep x along the baseline, so the same holds for them.

#include <Eigen/Core>
#include <imgui.h>

namespace einstar::app {

struct ImageFrame {
    ImVec2 origin;             // the drawn image's top left
    float scale = 1;           // screen px per image px
    float image_width = 1280;  // image px (its x extent, the drawn height)

    [[nodiscard]] ImVec2 at(float x, float y) const { return {origin.x + y * scale, origin.y + (image_width - x) * scale}; }
    [[nodiscard]] ImVec2 at(const Eigen::Vector2d& px) const { return at(static_cast<float>(px.x()), static_cast<float>(px.y())); }
    // The drawn size of an image_width x image_height image.
    [[nodiscard]] ImVec2 size(float image_height) const { return {image_height * scale, image_width * scale}; }
    // The image texture, turned to match at().
    void draw_image(ImDrawList* dl, ImTextureID tex, float image_height) const {
        const ImVec2 s = size(image_height);
        dl->AddImageQuad(tex, origin, ImVec2(origin.x + s.x, origin.y), ImVec2(origin.x + s.x, origin.y + s.y), ImVec2(origin.x, origin.y + s.y),
                         ImVec2(1, 0), ImVec2(1, 1), ImVec2(0, 1), ImVec2(0, 0));
    }
};

// An upright image laid out as an ImGui item of the drawn size (at the cursor), drawn with draw_image.
inline ImageFrame upright_image_item(ImTextureID tex, float image_width, float image_height, float drawn_width) {
    const ImageFrame f{ImGui::GetCursorScreenPos(), drawn_width / image_height, image_width};
    f.draw_image(ImGui::GetWindowDrawList(), tex, image_height);
    ImGui::Dummy(f.size(image_height));
    return f;
}

}  // namespace einstar::app
