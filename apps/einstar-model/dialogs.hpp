#pragma once

// macOS file dialogs.

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace einstar::modelapp {

// Extensions without the dot.
[[nodiscard]] std::optional<std::filesystem::path> choose_open_file(const std::vector<std::string>& extensions);
[[nodiscard]] std::vector<std::filesystem::path> choose_open_files(const std::vector<std::string>& extensions);
[[nodiscard]] std::optional<std::filesystem::path> choose_save_file(const std::string& name, const std::string& extension);

// What the clipboard holds for pasting photos: image files copied in the Finder, or an image's bytes (copied
// from Photos, a browser, a screenshot) as PNG / JPEG / HEIC / TIFF.
struct Pasted {
    std::vector<std::filesystem::path> files;
    std::string image_bytes;
};
[[nodiscard]] Pasted clipboard_images();

// Photo files ImageIO reads (extensions without the dot).
[[nodiscard]] const std::vector<std::string>& photo_extensions();

}  // namespace einstar::modelapp
