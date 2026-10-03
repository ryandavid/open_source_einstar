#pragma once

// The modelling app's preferences, kept outside the build and the app bundle:
// ~/Library/Application Support/Einstar/model-settings.json ($EINSTAR_SETTINGS_DIR overrides the folder).
// Unknown keys are kept, so a newer version's settings survive an older one.

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "einstar/core/error.hpp"

namespace einstar::model {

struct Settings {
    double tolerance_mm = 0.1;        // deviation within this is "on the model"
    double deviation_range_mm = 0.5;  // deviation shown fully saturated
    std::vector<std::string> recent;  // documents and scans, newest first
    nlohmann::json other = nlohmann::json::object();
};

[[nodiscard]] std::filesystem::path settings_dir();
[[nodiscard]] Settings load_settings();
Result<void> save_settings(const Settings& s);
void remember_recent(Settings& s, const std::filesystem::path& path);

}  // namespace einstar::model
