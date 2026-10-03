#pragma once

// Geometry as JSON, as the commands take and give it: vectors as [x, y, z] (mm), surfaces as
// {"kind": "plane", "normal": [..], "offset": ..}, {"kind": "cylinder", "point", "axis", "radius"}, ...

#include <vector>

#include <nlohmann/json.hpp>

#include "einstar/fit/surface.hpp"

namespace einstar::model {

[[nodiscard]] nlohmann::json vec_to_json(const Vec3& v);
[[nodiscard]] Vec3 vec_from_json(const nlohmann::json& j);  // throws nlohmann::json::exception
[[nodiscard]] nlohmann::json surface_to_json(const fit::Surface& s);
[[nodiscard]] fit::Surface surface_from_json(const nlohmann::json& j);
[[nodiscard]] std::vector<fit::SurfaceKind> kinds_from_json(const nlohmann::json& j);
[[nodiscard]] nlohmann::json frame_to_json(const SE3& T);
[[nodiscard]] SE3 frame_from_json(const nlohmann::json& j);

}  // namespace einstar::model
