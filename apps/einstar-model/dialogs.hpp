#pragma once

// macOS file dialogs.

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace einstar::modelapp {

// Extensions without the dot.
[[nodiscard]] std::optional<std::filesystem::path> choose_open_file(const std::vector<std::string>& extensions);
[[nodiscard]] std::optional<std::filesystem::path> choose_save_file(const std::string& name, const std::string& extension);

}  // namespace einstar::modelapp
