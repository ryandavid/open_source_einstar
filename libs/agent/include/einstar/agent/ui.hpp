#pragma once

// The agent's view of an ImGui UI. ImGui keeps no widget tree, so the inspector records every item as it is
// drawn (ImGui's test-engine hooks, IMGUI_ENABLE_TEST_ENGINE) and answers from the last complete frame.
// The input player feeds synthetic mouse and keyboard events through ImGui's own input queue, one step per
// frame, so everything that reads ImGui's input (widgets and the app's 3D view alike) sees them.

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <imgui.h>

#include "einstar/agent/protocol.hpp"

namespace einstar::agent {

struct UiItem {
    ImGuiID id = 0;
    std::string window;  // the root window's name
    std::string label;   // as given to the widget (may contain "##")
    ImVec2 min, max;     // window points
    bool checkable = false, checked = false, openable = false, opened = false, inputable = false;
    bool disabled = false, visible = true;

    [[nodiscard]] std::string ref() const;            // "#1a2b3c4d"
    [[nodiscard]] std::string visible_label() const;  // the label up to "##"
    [[nodiscard]] json to_json() const;
};

struct UiWindow {
    std::string name;
    ImVec2 pos, size;
    bool collapsed = false, focused = false;
};

class UiInspector {
public:
    UiInspector();
    ~UiInspector();
    // The frame drawn since the last call becomes the one answered from (call before ImGui::NewFrame).
    void end_frame();

    [[nodiscard]] const std::vector<UiItem>& items() const { return last_; }
    [[nodiscard]] const std::vector<UiWindow>& windows() const { return windows_; }
    // One item for a selector (protocol.hpp: kTarget), or not_found / ambiguous with the candidates.
    [[nodiscard]] std::variant<const UiItem*, Error> resolve(std::string_view selector) const;

    // From the ImGui hooks (ui.cpp).
    void on_item_add(ImGuiID id, const char* window, const ImVec2& min, const ImVec2& max, bool disabled, bool visible);
    void on_item_info(ImGuiID id, const char* label, int status_flags);

private:
    std::vector<UiItem> current_, last_;
    std::vector<UiWindow> windows_;
};

enum class Button { left = 0, right = 1, middle = 2 };

class InputPlayer {
public:
    // Each call queues steps and returns the step number that ends them (done() tells when played).
    std::uint64_t click(ImVec2 at, Button b, bool double_click, int modifiers);
    std::uint64_t move(ImVec2 at);
    std::uint64_t drag(const std::vector<ImVec2>& path, Button b, int modifiers);
    std::uint64_t scroll(std::optional<ImVec2> at, float dx, float dy);
    std::uint64_t key(ImGuiKey k, int modifiers);
    std::uint64_t type(std::string text);
    // Plays the next step into ImGui's input queue (call before ImGui::NewFrame).
    void step();
    [[nodiscard]] bool done(std::uint64_t until) const { return played_ >= until; }

    // "shift" / "ctrl" / "alt" / "super" -> ImGuiMod_ flags; an unknown name throws std::invalid_argument.
    static int parse_modifiers(const json& names);
    // An ImGui key by its name ("Escape", "Z", "Delete"), or ImGuiKey_None.
    static ImGuiKey key_by_name(std::string_view name);

private:
    struct Action {
        enum class Kind { pos, button, wheel, key, chars } kind;
        ImVec2 v{};
        int i = 0;  // button / key / modifier mask
        bool down = false;
        std::string text;
    };
    using Step = std::vector<Action>;
    std::uint64_t push(Step s);
    void modifiers(Step& s, int mods, bool down);

    std::deque<Step> steps_;
    std::uint64_t queued_ = 0, played_ = 0;
};

}  // namespace einstar::agent
