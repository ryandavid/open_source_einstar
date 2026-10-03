#pragma once

// A photo as an image to look at (the agent's view of it): scaled down, optionally cropped, with its
// annotations drawn and named as in the lists, and optionally a grid of pixel coordinates to point with.

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "einstar/core/error.hpp"
#include "einstar/model/photo.hpp"

namespace einstar::model {

struct PhotoRenderOptions {
    int max_size = 1600;                         // the longer side of the result
    std::optional<std::array<double, 4>> crop;   // x, y, width, height in photo pixels
    bool annotations = true;
    bool grid = false;                           // lines labelled with photo pixel coordinates
    std::vector<std::pair<Vec2, Vec2>> lines;    // more to draw (photo pixels), e.g. the model's edges
};

struct RenderedPhoto {
    std::string jpeg;
    int width = 0, height = 0;
    double scale = 1;                     // result pixels per photo pixel
    std::array<double, 4> crop{};         // the part of the photo shown
};

[[nodiscard]] Result<RenderedPhoto> render_photo(const Photo& photo, const PhotoRenderOptions& options = {});

}  // namespace einstar::model
