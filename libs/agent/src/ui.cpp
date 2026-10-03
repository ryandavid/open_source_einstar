#include "einstar/agent/ui.hpp"

#include <algorithm>
#include <cctype>
#include <format>
#include <stdexcept>

#include <imgui_internal.h>

namespace einstar::agent {
namespace {

UiInspector* g_inspector = nullptr;  // one ImGui context per app: the hooks report to it

bool iequals(std::string_view a, std::string_view b) {
    return a.size() == b.size() && std::ranges::equal(a, b, [](char x, char y) { return std::tolower(static_cast<unsigned char>(x)) == std::tolower(static_cast<unsigned char>(y)); });
}

bool icontains(std::string_view hay, std::string_view needle) {
    return !needle.empty() && std::ranges::search(hay, needle, [](char x, char y) {
                                  return std::tolower(static_cast<unsigned char>(x)) == std::tolower(static_cast<unsigned char>(y));
                              }).begin() != hay.end();
}

json candidates(const std::vector<const UiItem*>& items) {
    json c = json::array();
    for (const auto* i : items) c.push_back({{"ref", i->ref()}, {"window", i->window}, {"label", i->label}});
    return c;
}

}  // namespace

std::string UiItem::ref() const { return std::format("#{:08x}", id); }

std::string UiItem::visible_label() const {
    const auto hash = label.find("##");
    return hash == std::string::npos ? label : label.substr(0, hash);
}

json UiItem::to_json() const {
    json j = {{"ref", ref()}, {"window", window}, {"label", visible_label()}, {"rect", {min.x, min.y, max.x - min.x, max.y - min.y}}};
    if (visible_label() != label) j["full_label"] = label;
    if (checkable) j["checked"] = checked;
    if (openable) j["opened"] = opened;
    if (inputable) j["inputable"] = true;
    if (disabled) j["disabled"] = true;
    if (!visible) j["visible"] = false;
    return j;
}

UiInspector::UiInspector() { g_inspector = this; }
UiInspector::~UiInspector() {
    if (g_inspector == this) g_inspector = nullptr;
}

void UiInspector::end_frame() {
    last_ = std::move(current_);
    current_.clear();
    windows_.clear();
    ImGuiContext* g = ImGui::GetCurrentContext();
    if (!g) return;
    g->TestEngineHookItems = true;  // report items to the hooks below from now on
    for (ImGuiWindow* w : g->Windows) {
        if (w->ParentWindow || !w->Active || std::string_view(w->Name).starts_with("##")) continue;
        windows_.push_back({w->Name, w->Pos, w->Size, w->Collapsed, g->NavWindow && g->NavWindow->RootWindow == w});
    }
}

void UiInspector::on_item_add(ImGuiID id, const char* window, const ImVec2& min, const ImVec2& max, bool disabled, bool visible) {
    UiItem it;
    it.id = id;
    it.window = window ? window : "";
    it.min = min;
    it.max = max;
    it.disabled = disabled;
    it.visible = visible;
    current_.push_back(std::move(it));
}

void UiInspector::on_item_info(ImGuiID id, const char* label, int flags) {
    for (auto it = current_.rbegin(); it != current_.rend(); ++it) {
        if (it->id != id) continue;
        it->label = label ? label : "";
        it->checkable = (flags & ImGuiItemStatusFlags_Checkable) != 0;
        it->checked = (flags & ImGuiItemStatusFlags_Checked) != 0;
        it->openable = (flags & ImGuiItemStatusFlags_Openable) != 0;
        it->opened = (flags & ImGuiItemStatusFlags_Opened) != 0;
        it->inputable = (flags & ImGuiItemStatusFlags_Inputable) != 0;
        return;
    }
}

std::variant<const UiItem*, Error> UiInspector::resolve(std::string_view sel) const {
    std::vector<const UiItem*> found;
    if (sel.starts_with('#') && sel.size() > 1) {
        // A ref: the item's ImGui ID.
        std::uint32_t id = 0;
        try {
            id = static_cast<std::uint32_t>(std::stoul(std::string(sel.substr(1)), nullptr, 16));
        } catch (const std::exception&) {
            return error(ErrorCode::invalid_params, std::format("'{}' is not a ref (#hex)", sel));
        }
        for (const auto& i : last_)
            if (i.id == id) return &i;
        return error(ErrorCode::not_found, std::format("no item {} in the last frame (gone, or not drawn: a collapsed window?)", sel));
    }
    auto label_is = [](const UiItem& i, std::string_view l) { return i.label == l || i.visible_label() == l; };
    // "Window/Label": every split whose prefix is a window (labels may contain '/' themselves).
    bool windowed = false;
    for (std::size_t slash = sel.find('/'); slash != std::string_view::npos; slash = sel.find('/', slash + 1)) {
        const auto win = sel.substr(0, slash), label = sel.substr(slash + 1);
        if (std::ranges::none_of(last_, [&](const UiItem& i) { return i.window == win; })) continue;
        windowed = true;
        for (const auto& i : last_)
            if (i.window == win && label_is(i, label)) found.push_back(&i);
    }
    if (!windowed)
        for (const auto& i : last_)
            if (label_is(i, sel)) found.push_back(&i);
    if (found.size() == 1) return found.front();
    if (found.size() > 1)
        return error(ErrorCode::ambiguous, std::format("'{}' matches {} items: name one by its window ('Window/Label') or ref", sel, found.size()),
                     candidates(found));
    std::vector<const UiItem*> similar;
    const auto tail = sel.substr(sel.rfind('/') == std::string_view::npos ? 0 : sel.rfind('/') + 1);
    for (const auto& i : last_)
        if (similar.size() < 20 && (icontains(i.label, tail) || iequals(i.visible_label(), tail))) similar.push_back(&i);
    return error(ErrorCode::not_found, std::format("no item '{}' in the last frame", sel), candidates(similar));
}

// ---- input ----

std::uint64_t InputPlayer::push(Step s) {
    steps_.push_back(std::move(s));
    return ++queued_;
}

void InputPlayer::modifiers(Step& s, int mods, bool down) {
    for (const int m : {static_cast<int>(ImGuiMod_Ctrl), static_cast<int>(ImGuiMod_Shift), static_cast<int>(ImGuiMod_Alt), static_cast<int>(ImGuiMod_Super)})
        if (mods & m) s.push_back({Action::Kind::key, {}, m, down, {}});
}

std::uint64_t InputPlayer::click(ImVec2 at, Button b, bool double_click, int mods) {
    Step first{{Action::Kind::pos, at, 0, false, {}}};
    modifiers(first, mods, true);
    push(std::move(first));
    const int bi = static_cast<int>(b);
    for (int n = 0; n < (double_click ? 2 : 1); ++n) {
        push({{Action::Kind::button, {}, bi, true, {}}});
        push({{Action::Kind::button, {}, bi, false, {}}});
    }
    Step last;
    modifiers(last, mods, false);
    return push(std::move(last));
}

std::uint64_t InputPlayer::move(ImVec2 at) { return push({{Action::Kind::pos, at, 0, false, {}}}); }

std::uint64_t InputPlayer::drag(const std::vector<ImVec2>& path, Button b, int mods) {
    if (path.empty()) return queued_;
    Step first{{Action::Kind::pos, path.front(), 0, false, {}}};
    modifiers(first, mods, true);
    push(std::move(first));
    const int bi = static_cast<int>(b);
    push({{Action::Kind::button, {}, bi, true, {}}});
    for (std::size_t i = 1; i < path.size(); ++i) push({{Action::Kind::pos, path[i], 0, false, {}}});
    push({{Action::Kind::button, {}, bi, false, {}}});
    Step last;
    modifiers(last, mods, false);
    return push(std::move(last));
}

std::uint64_t InputPlayer::scroll(std::optional<ImVec2> at, float dx, float dy) {
    Step s;
    if (at) s.push_back({Action::Kind::pos, *at, 0, false, {}});
    s.push_back({Action::Kind::wheel, ImVec2(dx, dy), 0, false, {}});
    return push(std::move(s));
}

std::uint64_t InputPlayer::key(ImGuiKey k, int mods) {
    Step down;
    modifiers(down, mods, true);
    down.push_back({Action::Kind::key, {}, static_cast<int>(k), true, {}});
    push(std::move(down));
    Step up{{Action::Kind::key, {}, static_cast<int>(k), false, {}}};
    modifiers(up, mods, false);
    return push(std::move(up));
}

std::uint64_t InputPlayer::type(std::string text) { return push({{Action::Kind::chars, {}, 0, false, std::move(text)}}); }

void InputPlayer::step() {
    if (steps_.empty()) return;
    ImGuiIO& io = ImGui::GetIO();
    for (const auto& a : steps_.front()) switch (a.kind) {
            case Action::Kind::pos: io.AddMousePosEvent(a.v.x, a.v.y); break;
            case Action::Kind::button: io.AddMouseButtonEvent(a.i, a.down); break;
            case Action::Kind::wheel: io.AddMouseWheelEvent(a.v.x, a.v.y); break;
            case Action::Kind::key: io.AddKeyEvent(static_cast<ImGuiKey>(a.i), a.down); break;
            case Action::Kind::chars: io.AddInputCharactersUTF8(a.text.c_str()); break;
        }
    steps_.pop_front();
    ++played_;
}

int InputPlayer::parse_modifiers(const json& names) {
    int m = 0;
    if (names.is_null()) return 0;
    if (!names.is_array()) throw std::invalid_argument("modifiers must be an array of names");
    for (const auto& n : names) {
        const std::string s = n.is_string() ? n.get<std::string>() : std::string();
        if (s == "shift") m |= ImGuiMod_Shift;
        else if (s == "ctrl") m |= ImGuiMod_Ctrl;
        else if (s == "alt") m |= ImGuiMod_Alt;
        else if (s == "super") m |= ImGuiMod_Super;
        else throw std::invalid_argument(std::format("unknown modifier '{}' (shift, ctrl, alt, super)", s));
    }
    return m;
}

ImGuiKey InputPlayer::key_by_name(std::string_view name) {
    for (int k = ImGuiKey_NamedKey_BEGIN; k < ImGuiKey_NamedKey_END; ++k)
        if (iequals(ImGui::GetKeyName(static_cast<ImGuiKey>(k)), name)) return static_cast<ImGuiKey>(k);
    return ImGuiKey_None;
}

}  // namespace einstar::agent

// ---- ImGui's test-engine hooks (IMGUI_ENABLE_TEST_ENGINE): every item as it is drawn ----

void ImGuiTestEngineHook_ItemAdd(ImGuiContext* ctx, ImGuiID id, const ImRect& bb, const ImGuiLastItemData* item_data) {
    auto* in = einstar::agent::g_inspector;
    if (!in || id == 0 || !ctx->CurrentWindow) return;
    const ImGuiWindow* w = ctx->CurrentWindow;
    const bool disabled = item_data && (item_data->ItemFlags & ImGuiItemFlags_Disabled);
    in->on_item_add(id, w->RootWindow ? w->RootWindow->Name : w->Name, bb.Min, bb.Max, disabled, w->ClipRect.Overlaps(bb));
}

void ImGuiTestEngineHook_ItemInfo(ImGuiContext*, ImGuiID id, const char* label, ImGuiItemStatusFlags flags) {
    if (auto* in = einstar::agent::g_inspector) in->on_item_info(id, label, flags);
}

void ImGuiTestEngineHook_Log(ImGuiContext*, const char*, ...) {}

const char* ImGuiTestEngine_FindItemDebugLabel(ImGuiContext*, ImGuiID) { return nullptr; }
