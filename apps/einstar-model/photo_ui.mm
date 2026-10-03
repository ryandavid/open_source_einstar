#include "photo_ui.hpp"

#import <Metal/Metal.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <future>
#include <map>
#include <optional>
#include <set>
#include <string>

#include "dialogs.hpp"
#include "einstar/image/image.hpp"
#include "imgui.h"

namespace einstar::modelapp {

namespace {

constexpr int kThumbEdge = 320;     // decoded size of thumbnails
constexpr int kFullEdge = 4096;     // ... and of the photo being worked on (Metal's comfortable texture size)
constexpr float kThumb = 132;       // thumbnail size in the library, points
constexpr ImU32 kInk = IM_COL32(255, 214, 10, 255);
constexpr ImU32 kInkSelected = IM_COL32(255, 120, 40, 255);
constexpr ImU32 kShadow = IM_COL32(0, 0, 0, 190);
constexpr ImU32 kPending = IM_COL32(90, 220, 255, 255);

ImVec2 iv(const Vec2& v) { return {static_cast<float>(v.x()), static_cast<float>(v.y())}; }
Vec2 vv(const ImVec2& v) { return {v.x, v.y}; }

double segment_distance(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 d = b - a;
    const double t = d.squaredNorm() > 0 ? std::clamp((p - a).dot(d) / d.squaredNorm(), 0.0, 1.0) : 0.0;
    return (a + t * d - p).norm();
}

struct Texture {
    id<MTLTexture> texture = nil;
    int width = 0, height = 0;
    std::future<Result<image::Rgba>> pending;
    bool failed = false;
    int last_used = 0;
};

enum class Tool { select, dimension, diameter, angle, callout, note };
constexpr std::pair<Tool, const char*> kTools[] = {{Tool::select, "Select"},   {Tool::dimension, "Dimension"}, {Tool::diameter, "Diameter"},
                                                   {Tool::angle, "Angle"},     {Tool::callout, "Callout"},     {Tool::note, "Note"}};

model::AnnotationKind kind_of(Tool t) {
    switch (t) {
        case Tool::dimension: return model::AnnotationKind::dimension;
        case Tool::diameter: return model::AnnotationKind::diameter;
        case Tool::angle: return model::AnnotationKind::angle;
        case Tool::callout: return model::AnnotationKind::callout;
        case Tool::note:
        case Tool::select: return model::AnnotationKind::note;
    }
    return model::AnnotationKind::note;
}
// Points a tool takes from the user (a diameter: three on its rim).
int points_for(Tool t) {
    switch (t) {
        case Tool::dimension: return 2;
        case Tool::diameter: return 3;
        case Tool::angle: return 3;
        case Tool::callout: return 2;
        case Tool::note: return 1;
        case Tool::select: return 0;
    }
    return 0;
}
const char* hint_for(Tool t) {
    switch (t) {
        case Tool::dimension: return "Click both ends (Shift: level or plumb).";
        case Tool::diameter: return "Click three points on the rim.";
        case Tool::angle: return "Click a point on one side, the vertex, then a point on the other side.";
        case Tool::callout: return "Click what it points at, then where its text goes.";
        case Tool::note: return "Click where the note goes.";
        case Tool::select: return "Click an annotation to select it; drag its points to move them. Delete removes it.";
    }
    return "";
}

}  // namespace

struct PhotoUi::Impl {
    id<MTLDevice> device;
    std::map<std::string, Texture> textures;  // "<hash>/<edge>"
    int frame = 0;

    // The Photo window.
    int open_photo = 0;
    Tool tool = Tool::select;
    std::vector<Vec2> pending;  // points placed for the annotation being made
    double zoom = 1;            // screen points per photo pixel
    Vec2 pan = Vec2::Zero();    // the photo pixel at the canvas centre
    bool fit = true;
    int selected = 0;           // annotation id
    struct Drag {
        int annotation = 0;
        std::size_t point = 0;
        std::vector<Vec2> points;
    };
    std::optional<Drag> drag;

    // The annotation popup (new or editing).
    struct Edit {
        bool open = false;
        int annotation = 0;  // 0: new
        model::AnnotationKind kind = model::AnnotationKind::dimension;
        std::vector<Vec2> points;
        char value[96] = {};
        char text[512] = {};
        std::set<int> links;
        std::string error;
    } edit;

    char note_text[512] = {};
    char caption[512] = {};
    int caption_for = 0;
    char rename[128] = {};
    int rename_for = 0;

    // ---- textures ----

    Texture* texture(const model::PhotoBlob& blob, const std::shared_ptr<const model::PhotoBlob>& keep, int edge) {
        const std::string key = std::format("{}/{}", blob.hash, edge);
        auto [it, fresh] = textures.try_emplace(key);
        Texture& t = it->second;
        t.last_used = frame;
        if (fresh) t.pending = std::async(std::launch::async, [keep, edge] { return image::decode(keep->bytes, edge); });
        if (t.pending.valid() && t.pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto img = t.pending.get();
            if (!img) {
                t.failed = true;
            } else {
                MTLTextureDescriptor* d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                            width:NSUInteger(img->width)
                                                                                           height:NSUInteger(img->height)
                                                                                        mipmapped:NO];
                d.usage = MTLTextureUsageShaderRead;
                t.texture = [device newTextureWithDescriptor:d];
                [t.texture replaceRegion:MTLRegionMake2D(0, 0, NSUInteger(img->width), NSUInteger(img->height))
                             mipmapLevel:0
                               withBytes:img->pixels.data()
                             bytesPerRow:NSUInteger(img->width) * 4];
                t.width = img->width;
                t.height = img->height;
            }
        }
        return t.texture ? &t : nullptr;
    }
    // Full-size textures not drawn for a while are let go (thumbnails stay).
    void trim() {
        std::erase_if(textures, [&](const auto& kv) {
            return kv.first.ends_with(std::format("/{}", kFullEdge)) && frame - kv.second.last_used > 120 && !kv.second.pending.valid();
        });
    }

    static ImTextureID tex_id(const Texture& t) { return (ImTextureID)(__bridge void*)t.texture; }

    static const model::Photo* find(const ModelApp& app, int id) {
        for (const auto& ph : app.doc.state().photos)
            if (ph.id == id) return &ph;
        return nullptr;
    }

    // ---- the library ----

    void draw_library(ModelApp& app) {
        const ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 330, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(320, io.DisplaySize.y * 0.55f), ImGuiCond_FirstUseEver);
        ImGui::Begin("Photos");
        if (app.busy()) {
            ImGui::TextDisabled("(busy: %s)", app.busy_text().c_str());
            ImGui::End();
            return;
        }
        if (!app.doc.has_scan()) {
            ImGui::TextWrapped("Open a scan first: photos are kept with its model.");
            ImGui::End();
            return;
        }
        if (ImGui::Button("Import...")) {
            const auto files = choose_open_files(photo_extensions());
            if (!files.empty()) import(app, files);
        }
        ImGui::SameLine();
        if (ImGui::Button("Paste")) paste(app);
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Photos of the part, taken anywhere: they need not show where it was scanned.\n"
                              "Import them, drop them on the window, or paste one (Cmd+V).\n"
                              "Open one to mark measurements and remarks on it; the agent reads them too.");

        const auto& photos = app.doc.state().photos;
        if (photos.empty()) ImGui::TextWrapped("No photos yet.");
        const float avail = ImGui::GetContentRegionAvail().x;
        const int columns = std::max(1, static_cast<int>(avail / (kThumb + 8)));
        int col = 0;
        for (const auto& ph : photos) {
            ImGui::PushID(ph.id);
            ImGui::BeginGroup();
            const Texture* t = ph.blob ? texture(*ph.blob, ph.blob, kThumbEdge) : nullptr;
            const ImVec2 box(kThumb, kThumb * 0.75f);
            if (t) {
                // Fitted into the box, aspect kept.
                const float s = std::min(box.x / static_cast<float>(t->width), box.y / static_cast<float>(t->height));
                const ImVec2 size(static_cast<float>(t->width) * s, static_cast<float>(t->height) * s);
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (box.x - size.x) / 2);
                if (ImGui::ImageButton("thumb", tex_id(*t), size)) open(ph.id);
            } else {
                ImGui::Button(ph.blob && textures[std::format("{}/{}", ph.blob->hash, kThumbEdge)].failed ? "(unreadable)" : "...", box);
            }
            if (ImGui::BeginPopupContextItem("photo menu")) {
                if (ImGui::MenuItem("Open")) open(ph.id);
                if (ImGui::MenuItem("Rename...")) {
                    rename_for = ph.id;
                    std::snprintf(rename, sizeof(rename), "%s", ph.name.c_str());
                }
                if (ImGui::MenuItem("Delete")) app.run("photo.delete", {{"photo", ph.id}});
                ImGui::EndPopup();
            }
            if (rename_for == ph.id) {
                ImGui::SetNextItemWidth(kThumb);
                ImGui::SetKeyboardFocusHere();
                if (ImGui::InputText("##rename", rename, sizeof(rename), ImGuiInputTextFlags_EnterReturnsTrue)) {
                    app.run("photo.update", {{"photo", ph.id}, {"name", std::string(rename)}});
                    rename_for = 0;
                }
                if (ImGui::IsKeyPressed(ImGuiKey_Escape)) rename_for = 0;
            } else {
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + kThumb);
                ImGui::TextUnformatted(ph.name.c_str());
                ImGui::PopTextWrapPos();
                ImGui::TextDisabled("%zu marked%s", ph.annotations.size(), ph.camera ? ", registered" : "");
            }
            ImGui::EndGroup();
            ImGui::PopID();
            if (++col < columns) ImGui::SameLine();
            else col = 0;
        }
        if (col != 0) ImGui::NewLine();

        ImGui::SeparatorText("Notes about the part");
        int remove = 0;
        for (const auto& n : app.doc.state().notes) {
            ImGui::PushID(n.id);
            if (ImGui::SmallButton("x")) remove = n.id;
            ImGui::SameLine();
            ImGui::TextWrapped("%s", n.text.c_str());
            if (n.author != "user" && ImGui::IsItemHovered()) ImGui::SetTooltip("by the %s", n.author.c_str());
            ImGui::PopID();
        }
        if (remove) app.run("note.delete", {{"note", remove}});
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputTextWithHint("##note", "material, finish, threads, what it mates with ... (Enter)", note_text, sizeof(note_text),
                                     ImGuiInputTextFlags_EnterReturnsTrue) &&
            note_text[0]) {
            if (app.run("note.add", {{"text", std::string(note_text)}}).ok) note_text[0] = 0;
        }
        ImGui::End();
    }

    void open(int id) {
        if (open_photo != id) {
            open_photo = id;
            fit = true;
            pending.clear();
            selected = 0;
            drag.reset();
        }
        ImGui::SetWindowFocus("Photo");
    }

    void import(ModelApp& app, const std::vector<std::filesystem::path>& files) {
        json paths = json::array();
        for (const auto& f : files) paths.push_back(f.string());
        const auto o = app.run("photo.import", {{"paths", paths}});
        if (o.ok && !o.result["photos"].empty()) open(o.result["photos"].back()["id"].get<int>());
    }

    void paste(ModelApp& app) {
        if (!app.doc.has_scan() || app.busy()) return;
        const Pasted p = clipboard_images();
        if (!p.files.empty()) return import(app, p.files);
        if (p.image_bytes.empty()) return app.set_status("paste: the clipboard holds no image", true);
        // A copied image has no file: kept as a JPEG (TIFF from Photos would be many times larger), as it is otherwise.
        std::string bytes = p.image_bytes;
        if (const auto info = image::probe(bytes); info && info->mime == "image/tiff")
            if (const auto img = image::decode(bytes)) bytes = image::encode_jpeg(*img, 0.92);
        const auto o = app.run("photo.import", {{"data_base64", model::base64_encode(bytes)}, {"name", "pasted"}});
        if (o.ok) open(o.result["photos"].back()["id"].get<int>());
    }

    // ---- the photo window ----

    void draw_photo(ModelApp& app) {
        if (!open_photo) return;
        const ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(500, 40), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(std::min(1000.0f, io.DisplaySize.x - 520), io.DisplaySize.y - 120), ImGuiCond_FirstUseEver);
        bool keep_open = true;
        if (!ImGui::Begin("Photo", &keep_open, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse)) {
            ImGui::End();
            return;
        }
        if (!keep_open || app.busy() || !app.doc.has_scan() || !find(app, open_photo)) {
            if (!keep_open || !app.doc.has_scan() || (!app.busy() && !find(app, open_photo))) open_photo = 0;
            ImGui::End();
            return;
        }
        const model::Photo& ph = *find(app, open_photo);

        // Tools.
        for (const auto& [t, name] : kTools) {
            if (t != Tool::select) ImGui::SameLine();
            if (ImGui::RadioButton(name, tool == t)) {
                tool = t;
                pending.clear();
            }
        }
        ImGui::SameLine(0, 24);
        if (ImGui::Button("Fit")) fit = true;
        ImGui::TextDisabled("%s  %s  (wheel: zoom, right-drag: pan, Esc: cancel)", ph.name.c_str(), hint_for(tool));

        const float side = 300;
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        ImGui::BeginChild("canvas", ImVec2(std::max(100.0f, avail.x - side - 8), avail.y), ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        draw_canvas(app, ph);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("side", ImVec2(0, avail.y));
        draw_side(app, ph);
        ImGui::EndChild();
        draw_edit_popup(app);
        ImGui::End();
    }

    void draw_canvas(ModelApp& app, const model::Photo& ph) {
        const ImGuiIO& io = ImGui::GetIO();
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const ImVec2 size = ImGui::GetContentRegionAvail();
        if (size.x < 10 || size.y < 10) return;
        ImGui::InvisibleButton("##canvas", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
        const bool hovered = ImGui::IsItemHovered();
        const Vec2 center = vv(p0) + 0.5 * vv(size);
        if (fit) {
            zoom = 0.98 * std::min(static_cast<double>(size.x) / ph.width, static_cast<double>(size.y) / ph.height);
            pan = Vec2(ph.width / 2.0, ph.height / 2.0);
            fit = false;
        }
        const auto screen = [&](const Vec2& px) { return center + (px - pan) * zoom; };
        const auto photo_at = [&](const Vec2& s) { return pan + (s - center) / zoom; };
        const Vec2 mouse = vv(io.MousePos);

        // Zoom about the cursor, pan with the right or middle button.
        if (hovered && io.MouseWheel != 0) {
            const Vec2 before = photo_at(mouse);
            zoom = std::clamp(zoom * std::pow(1.15, static_cast<double>(io.MouseWheel)), 0.02, 40.0);
            pan = before - (mouse - center) / zoom;
        }
        if (ImGui::IsItemActive() && (ImGui::IsMouseDragging(ImGuiMouseButton_Right) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle)))
            pan -= vv(io.MouseDelta) / zoom;

        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->PushClipRect(p0, ImVec2(p0.x + size.x, p0.y + size.y), true);
        const Texture* t = ph.blob ? texture(*ph.blob, ph.blob, kFullEdge) : nullptr;
        if (t) dl->AddImage(tex_id(*t), iv(screen({0, 0})), iv(screen({static_cast<double>(ph.width), static_cast<double>(ph.height)})));
        else dl->AddText(p0, IM_COL32(200, 200, 200, 255), "Loading ...");

        // Where a new point would go (Shift: level or plumb with the previous point).
        Vec2 at = photo_at(mouse);
        at = Vec2(std::clamp(at.x(), 0.0, static_cast<double>(ph.width)), std::clamp(at.y(), 0.0, static_cast<double>(ph.height)));
        if (io.KeyShift && !pending.empty() && tool == Tool::dimension) {
            const Vec2 d = at - pending.back();
            at = std::abs(d.x()) > std::abs(d.y()) ? Vec2(at.x(), pending.back().y()) : Vec2(pending.back().x(), at.y());
        }

        // Annotations.
        const auto& annotations = ph.annotations;
        for (const auto& a : annotations) {
            std::vector<Vec2> pts = a.points;
            if (drag && drag->annotation == a.id) pts = drag->points;
            draw_annotation(dl, a, pts, screen, a.id == selected);
        }
        // The annotation being made.
        if (tool != Tool::select && hovered) {
            std::vector<Vec2> pts = pending;
            pts.push_back(at);
            for (std::size_t i = 0; i + 1 < pts.size(); ++i) dl->AddLine(iv(screen(pts[i])), iv(screen(pts[i + 1])), kPending, 2);
            for (const auto& q : pts) dl->AddCircle(iv(screen(q)), 4, kPending, 0, 2);
        }

        // Clicks.
        if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            if (tool != Tool::select) {
                pending.push_back(at);
                if (static_cast<int>(pending.size()) == points_for(tool)) {
                    begin_edit(0, kind_of(tool), pending);
                    pending.clear();
                }
            } else {
                // A point to drag, else the nearest annotation within reach.
                selected = 0;
                double best = 10;
                for (const auto& a : annotations) {
                    for (std::size_t i = 0; i < a.points.size(); ++i)
                        if (const double dpx = (screen(a.points[i]) - mouse).norm(); dpx < best) {
                            best = dpx;
                            selected = a.id;
                            drag = Drag{a.id, i, a.points};
                        }
                }
                if (!selected) {
                    drag.reset();
                    for (const auto& a : annotations)
                        if (const double dpx = annotation_distance(a, mouse, screen); dpx < best) {
                            best = dpx;
                            selected = a.id;
                        }
                }
            }
        }
        if (drag) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 2)) drag->points[drag->point] = at;
            } else {
                // Released: one undo step for the whole move.
                const auto* a = annotation(ph, drag->annotation);
                if (a && a->points != drag->points) {
                    json pts = json::array();
                    for (const auto& q : drag->points) pts.push_back({q.x(), q.y()});
                    app.run("photo.annotation.update", {{"annotation", drag->annotation}, {"points", pts}});
                }
                drag.reset();
            }
        }
        if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !io.WantTextInput) {
            if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
                pending.clear();
                selected = 0;
            }
            if (selected && (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace))) {
                app.run("photo.annotation.delete", {{"annotation", selected}});
                selected = 0;
            }
        }

        // A loupe while placing points: 4x around the cursor.
        if (tool != Tool::select && hovered && t) {
            const float box = 150, mag = 4;
            const ImVec2 b0(static_cast<float>(mouse.x()) + 24, static_cast<float>(mouse.y()) + 24);
            const double half = box / (2 * zoom * mag);  // photo px either side
            const Vec2 c = at;
            const ImVec2 uv0(static_cast<float>((c.x() - half) / ph.width), static_cast<float>((c.y() - half) / ph.height));
            const ImVec2 uv1(static_cast<float>((c.x() + half) / ph.width), static_cast<float>((c.y() + half) / ph.height));
            dl->AddRectFilled(b0, ImVec2(b0.x + box, b0.y + box), IM_COL32(0, 0, 0, 255));
            dl->AddImage(tex_id(*t), b0, ImVec2(b0.x + box, b0.y + box), uv0, uv1);
            dl->AddLine(ImVec2(b0.x + box / 2, b0.y), ImVec2(b0.x + box / 2, b0.y + box), kPending, 1);
            dl->AddLine(ImVec2(b0.x, b0.y + box / 2), ImVec2(b0.x + box, b0.y + box / 2), kPending, 1);
            dl->AddRect(b0, ImVec2(b0.x + box, b0.y + box), IM_COL32(255, 255, 255, 200));
        }
        dl->PopClipRect();
    }

    static const model::Annotation* annotation(const model::Photo& ph, int id) {
        for (const auto& a : ph.annotations)
            if (a.id == id) return &a;
        return nullptr;
    }

    template <class Screen>
    static double annotation_distance(const model::Annotation& a, const Vec2& mouse, const Screen& screen) {
        double best = 1e9;
        for (std::size_t i = 0; i + 1 < a.points.size(); ++i) best = std::min(best, segment_distance(mouse, screen(a.points[i]), screen(a.points[i + 1])));
        if (const auto c = model::annotation_circle(a)) {
            const Vec2 sc = screen(c->first);
            const double r = (screen(c->first + Vec2(c->second, 0)) - sc).norm();
            best = std::min(best, std::abs((mouse - sc).norm() - r));
        }
        for (const auto& q : a.points) best = std::min(best, (screen(q) - mouse).norm());
        return best;
    }

    template <class Screen>
    static void draw_annotation(ImDrawList* dl, const model::Annotation& a, const std::vector<Vec2>& points, const Screen& screen, bool selected) {
        const ImU32 ink = selected ? kInkSelected : kInk;
        const float w = selected ? 3.0f : 2.0f;
        const auto line = [&](const Vec2& p, const Vec2& q) {
            dl->AddLine(iv(p), iv(q), kShadow, w + 2);
            dl->AddLine(iv(p), iv(q), ink, w);
        };
        std::vector<Vec2> s;
        for (const auto& q : points) s.push_back(screen(q));
        Vec2 label_at = s.empty() ? Vec2::Zero() : s[0];
        model::Annotation shown = a;
        shown.points = points;
        switch (a.kind) {
            case model::AnnotationKind::dimension:
                if (s.size() == 2) {
                    line(s[0], s[1]);
                    const Vec2 d = s[1] - s[0];
                    const Vec2 n = d.norm() > 1e-6 ? Vec2(Vec2(-d.y(), d.x()).normalized() * 7.0) : Vec2(Vec2::Zero());
                    line(s[0] - n, s[0] + n);
                    line(s[1] - n, s[1] + n);
                    label_at = 0.5 * (s[0] + s[1]);
                }
                break;
            case model::AnnotationKind::diameter:
                if (const auto c = model::annotation_circle(shown)) {
                    const Vec2 sc = screen(c->first);
                    const float r = static_cast<float>((screen(c->first + Vec2(c->second, 0)) - sc).norm());
                    dl->AddCircle(iv(sc), r, kShadow, 0, w + 2);
                    dl->AddCircle(iv(sc), r, ink, 0, w);
                    label_at = sc + Vec2(r * 0.7, -r * 0.7);
                }
                break;
            case model::AnnotationKind::angle:
                if (s.size() == 3) {
                    line(s[1], s[0]);
                    line(s[1], s[2]);
                    label_at = s[1];
                }
                break;
            case model::AnnotationKind::callout:
                if (s.size() == 2) {
                    line(s[1], s[0]);
                    const Vec2 d = s[0] - s[1];
                    if (d.norm() > 1e-6) {
                        const Vec2 u = d.normalized(), n(-u.y(), u.x());
                        line(s[0], s[0] - 12 * u + 6 * n);
                        line(s[0], s[0] - 12 * u - 6 * n);
                    }
                    label_at = s[1];
                }
                break;
            case model::AnnotationKind::note: break;
        }
        for (const auto& q : s) {
            dl->AddCircleFilled(iv(q), w + 2.5f, kShadow);
            dl->AddCircleFilled(iv(q), w + 1.0f, ink);
        }
        const std::string text = model::annotation_summary(a);
        const ImVec2 ts = ImGui::CalcTextSize(text.c_str());
        const ImVec2 t0(static_cast<float>(label_at.x()) + 8, static_cast<float>(label_at.y()) + 6);
        dl->AddRectFilled(ImVec2(t0.x - 3, t0.y - 2), ImVec2(t0.x + ts.x + 3, t0.y + ts.y + 2), IM_COL32(20, 20, 20, 210), 3);
        dl->AddText(t0, ink, text.c_str());
    }

    void draw_side(ModelApp& app, const model::Photo& ph) {
        // Caption.
        if (caption_for != ph.id) {
            caption_for = ph.id;
            std::snprintf(caption, sizeof(caption), "%s", ph.caption.c_str());
        }
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##caption", "caption: what this photo shows", caption, sizeof(caption));
        if (ImGui::IsItemDeactivatedAfterEdit()) app.run("photo.update", {{"photo", ph.id}, {"caption", std::string(caption)}});
        ImGui::TextDisabled("%d x %d px%s", ph.width, ph.height,
                            ph.exif.contains("model") ? std::format(", {}", ph.exif["model"].get<std::string>()).c_str() : "");

        ImGui::SeparatorText("Marked on this photo");
        if (ph.annotations.empty()) ImGui::TextWrapped("Nothing yet: pick a tool above and click on the photo.");
        int remove = 0;
        for (const auto& a : ph.annotations) {
            ImGui::PushID(a.id);
            if (ImGui::Selectable(model::annotation_summary(a).c_str(), selected == a.id, ImGuiSelectableFlags_AllowDoubleClick)) {
                selected = a.id;
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) begin_edit(a.id, a.kind, a.points, &a);
            }
            if (ImGui::BeginPopupContextItem("annotation menu")) {
                if (ImGui::MenuItem("Edit...")) begin_edit(a.id, a.kind, a.points, &a);
                if (ImGui::MenuItem("Delete")) remove = a.id;
                ImGui::EndPopup();
            }
            if (!a.links.empty() || a.author != "user") {
                std::string about;
                for (const int id : a.links) about += (about.empty() ? "about " : ", ") + entity_name(app, id);
                if (a.author != "user") about += (about.empty() ? "by the " : "; by the ") + a.author;
                ImGui::Indent();
                ImGui::TextDisabled("%s", about.c_str());
                ImGui::Unindent();
            }
            ImGui::PopID();
        }
        if (remove) app.run("photo.annotation.delete", {{"annotation", remove}});
        if (selected && annotation(ph, selected)) {
            if (ImGui::Button("Edit...")) {
                const auto* a = annotation(ph, selected);
                begin_edit(a->id, a->kind, a->points, a);
            }
            ImGui::SameLine();
            if (ImGui::Button("Delete")) {
                app.run("photo.annotation.delete", {{"annotation", selected}});
                selected = 0;
            }
        }
    }

    static std::string entity_name(const ModelApp& app, int id) {
        const auto& s = app.doc.state();
        for (const auto& l : s.labels)
            if (l.id == id) return l.name;
        for (const auto& h : s.holes)
            if (h.id == id) return h.name;
        for (const auto& f : s.fillets)
            if (f.id == id) return f.name;
        return std::format("#{}", id);
    }

    void begin_edit(int id, model::AnnotationKind kind, std::vector<Vec2> points, const model::Annotation* a = nullptr) {
        edit = Edit{};
        edit.open = true;
        edit.annotation = id;
        edit.kind = kind;
        edit.points = std::move(points);
        if (a) {
            const std::string v = !a->entered.empty() ? a->entered : a->value ? std::format("{:g}", *a->value) : std::string();
            std::snprintf(edit.value, sizeof(edit.value), "%s", v.c_str());
            std::snprintf(edit.text, sizeof(edit.text), "%s", a->text.c_str());
            edit.links.insert(a->links.begin(), a->links.end());
        }
    }

    void draw_edit_popup(ModelApp& app) {
        if (edit.open) {
            ImGui::OpenPopup("Annotation");
            edit.open = false;
        }
        ImGui::SetNextWindowSize(ImVec2(420, 0), ImGuiCond_Appearing);
        if (!ImGui::BeginPopupModal("Annotation", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
        const bool angle = edit.kind == model::AnnotationKind::angle;
        ImGui::Text("%s %s", edit.annotation ? "Edit" : "New", std::string(model::annotation_kind_name(edit.kind)).c_str());
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        bool enter = ImGui::InputTextWithHint("Value", angle ? "30, 30°" : "42, 42 mm, 1 1/2\", Ø6 ±0.02, R2", edit.value, sizeof(edit.value),
                                              ImGuiInputTextFlags_EnterReturnsTrue);
        if (edit.value[0]) {
            if (const auto v = model::parse_value(edit.value, angle)) {
                const double shown = edit.kind == model::AnnotationKind::diameter && v->form == 'R' ? 2 * v->value : v->value;
                ImGui::TextDisabled("= %s%s", v->angle ? std::format("{:.2f}°", shown).c_str() : std::format("{:.3f} mm", shown).c_str(),
                                    v->tolerance ? std::format(" ±{:g}", *v->tolerance).c_str() : "");
            } else {
                ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "not a value");
            }
        }
        enter |= ImGui::InputTextWithHint("Text", "what it is, e.g. 'flange width', 'M6 thread'", edit.text, sizeof(edit.text),
                                          ImGuiInputTextFlags_EnterReturnsTrue);
        // What it is about.
        const auto& s = app.doc.state();
        std::string about;
        for (const int id : edit.links) about += (about.empty() ? "" : ", ") + entity_name(app, id);
        if (ImGui::BeginCombo("About", about.empty() ? "(nothing linked)" : about.c_str(), ImGuiComboFlags_HeightLarge)) {
            const auto toggle = [&](int id, const std::string& name, const char* what) {
                bool on = edit.links.contains(id);
                if (ImGui::Checkbox(std::format("{}  ({})##{}", name, what, id).c_str(), &on)) {
                    if (on) edit.links.insert(id);
                    else edit.links.erase(id);
                }
            };
            for (const auto& l : s.labels)
                if (l.role != model::Role::ignore) toggle(l.id, l.name, std::string(model::role_name(l.role)).c_str());
            for (const auto& h : s.holes) toggle(h.id, h.name, "hole");
            for (const auto& f : s.fillets) toggle(f.id, f.name, "fillet");
            ImGui::EndCombo();
        }
        if (!edit.error.empty()) ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%s", edit.error.c_str());
        if (ImGui::Button("OK", ImVec2(120, 0)) || enter) {
            json p = {{"text", std::string(edit.text)}};
            p["value"] = edit.value[0] ? json(std::string(edit.value)) : json(nullptr);
            p["links"] = json::array();
            for (const int id : edit.links) p["links"].push_back(id);
            if (edit.annotation) {
                p["annotation"] = edit.annotation;
            } else {
                p["photo"] = open_photo;
                p["kind"] = std::string(model::annotation_kind_name(edit.kind));
                p["points"] = json::array();
                for (const auto& q : edit.points) p["points"].push_back({q.x(), q.y()});
                if (!edit.value[0]) p.erase("value");
            }
            const auto o = app.run(edit.annotation ? "photo.annotation.update" : "photo.annotate", p);
            if (o.ok) {
                if (!edit.annotation) selected = o.result["id"].get<int>();
                ImGui::CloseCurrentPopup();
            } else {
                edit.error = o.error;
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
};

PhotoUi::PhotoUi(void* metal_device) : impl_(std::make_unique<Impl>()) { impl_->device = (__bridge id<MTLDevice>)metal_device; }
PhotoUi::~PhotoUi() = default;

void PhotoUi::import_files(ModelApp& app, const std::vector<std::filesystem::path>& files) {
    if (!files.empty() && app.doc.has_scan() && !app.busy()) impl_->import(app, files);
}
void PhotoUi::paste(ModelApp& app) { impl_->paste(app); }
void PhotoUi::show(int photo_id) { impl_->open(photo_id); }

void PhotoUi::draw(ModelApp& app) {
    ++impl_->frame;
    impl_->draw_library(app);
    impl_->draw_photo(app);
    impl_->trim();
}

}  // namespace einstar::modelapp
