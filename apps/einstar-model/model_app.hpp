#pragma once

// The modelling app's state: the document, how it is shown, the brush, and work that runs in the background
// (processing an .estr). Everything that changes the document goes through ModelApp::run (the document's
// commands), from the panels, the brush and the agent alike.

#include <filesystem>
#include <future>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "einstar/model/document.hpp"
#include "einstar/model/settings.hpp"
#include "einstar/render/types.hpp"
#include "einstar/render/view_camera.hpp"

namespace einstar::modelapp {

using model::json;

enum class Display { labels, deviation, model };
[[nodiscard]] std::string_view display_name(Display d);
[[nodiscard]] std::optional<Display> display_from_name(std::string_view s);

// What the renderer should show; geometry only when it changed (colours change more often).
struct RenderUpdate {
    std::optional<std::pair<std::vector<render::MeshVertex>, std::vector<std::uint32_t>>> geometry;
    std::vector<render::Rgba8> colors;
    std::vector<render::LineVertex> lines;
};

class ModelApp {
public:
    ModelApp();

    model::Document doc;
    model::Settings settings;
    Display display = Display::labels;
    bool show_edges = true;
    int active_label = 0;      // the brush paints this label
    float brush_radius = 2.0f;

    // A document command from the UI (or the brush). The outcome also becomes the status line.
    model::Outcome run(std::string_view command, const json& params, model::Author author = model::Author::user);

    // Opening a scan can take minutes (an .estr is processed first): it runs in the background.
    void open(const std::filesystem::path& path, bool fine = false);
    [[nodiscard]] bool busy() const { return job_.valid(); }
    [[nodiscard]] const std::string& busy_text() const { return busy_text_; }
    // Called every frame: finishes background work.
    void update();
    // The outcome of the last background open, once (for the agent waiting on it).
    std::optional<model::Outcome> take_open_outcome() { return std::exchange(open_outcome_, std::nullopt); }

    // The brush: a stroke (one undo step) of dabs where the ray meets the scan.
    void begin_stroke(bool erase);
    void dab(const Vec3f& origin, const Vec3f& direction);
    void end_stroke();
    [[nodiscard]] bool stroking() const { return stroke_.has_value(); }

    // The label under a ray, for the hover hint.
    [[nodiscard]] std::optional<std::string> label_at(const Vec3f& origin, const Vec3f& direction) const;

    // Camera: frame the scan or a label; look from a named direction (in the first datum's frame if there is
    // one): "top", "bottom", "front", "back", "left", "right", "iso".
    void frame(render::ViewCamera& camera, const std::string& label = {}) const;
    bool look_from(render::ViewCamera& camera, std::string_view preset) const;

    std::optional<RenderUpdate> take_render_update();

    // The panels.
    void draw_ui(render::ViewCamera& camera);

    [[nodiscard]] const std::string& status() const { return status_; }
    [[nodiscard]] bool status_is_error() const { return status_error_; }

private:
    [[nodiscard]] render::Rgba8 label_color(int id) const;

    std::future<Result<std::pair<recon::TriangleMesh, model::ScanSource>>> job_;
    std::filesystem::path job_path_;
    std::string busy_text_;
    std::optional<model::Outcome> open_outcome_;
    struct Stroke {
        bool erase = false;
        int dabs = 0;
    };
    std::optional<Stroke> stroke_;
    std::string status_;
    bool status_error_ = false;
    // What the renderer has.
    std::uint64_t shown_revision_ = ~0ull;
    Display shown_display_ = Display::labels;
    bool shown_edges_ = false;
    double shown_tolerance_ = -1, shown_range_ = -1;
    bool shown_model_geometry_ = false;  // the renderer holds the model's tessellation, not the scan
    const recon::TriangleMesh* shown_scan_ = nullptr;

    // Panel state.
    struct ConstraintForm {
        int type = 0;
        int a = 0, b = 0, datum = 0, axis = 2;
        double value = 0;
    } form_;
    struct DatumForm {
        int z = 0, x = 0;
    } datum_form_;
    struct PlaneForm {
        int datum = 0, axis = 2;
        double offset = 0;
        bool facing_minus = true;
    } plane_form_;
    char rename_[128] = {};
    int rename_for_ = -1;
};

}  // namespace einstar::modelapp
