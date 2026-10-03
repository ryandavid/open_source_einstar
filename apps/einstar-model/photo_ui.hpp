#pragma once

// The photo library's windows: "Photos" (the photos as thumbnails, importing, the part's notes) and "Photo" (one
// photo to look at closely and mark up: dimensions, diameters, angles, callouts, notes). Every change goes
// through the document's commands (ModelApp::run), so it is undoable and the agent sees it.

#include <filesystem>
#include <memory>
#include <vector>

#include "einstar/render/view_camera.hpp"
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
    // The windows, and over the 3D view: the matched points and the registered photos' cameras.
    void draw(ModelApp& app, render::ViewCamera& camera);
    // Matching points: a pixel has been picked in the photo and waits for its point on the scan (a click in the
    // 3D view, given here).
    [[nodiscard]] bool awaiting_scan_point() const;
    void scan_point(ModelApp& app, const Vec3& point);
    // Opens a photo in the Photo window.
    void show(int photo_id);

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace einstar::modelapp
