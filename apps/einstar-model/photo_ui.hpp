#pragma once

// The photo library's windows: "Photos" (the photos as thumbnails, importing, the part's notes) and "Photo" (one
// photo to look at closely and mark up: dimensions, diameters, angles, callouts, notes). Every change goes
// through the document's commands (ModelApp::run), so it is undoable and the agent sees it.

#include <filesystem>
#include <memory>
#include <vector>

#include "model_app.hpp"

namespace einstar::modelapp {

class PhotoUi {
public:
    explicit PhotoUi(void* metal_device);  // id<MTLDevice>
    ~PhotoUi();
    PhotoUi(const PhotoUi&) = delete;
    PhotoUi& operator=(const PhotoUi&) = delete;

    void import_files(ModelApp& app, const std::vector<std::filesystem::path>& files);
    // Cmd+V: image files copied in the Finder, or an image copied from Photos, a browser or a screenshot.
    void paste(ModelApp& app);
    void draw(ModelApp& app);
    // Opens a photo in the Photo window.
    void show(int photo_id);

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace einstar::modelapp
