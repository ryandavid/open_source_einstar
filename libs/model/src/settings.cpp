#include "einstar/model/settings.hpp"

#include <cstdlib>
#include <fstream>

namespace einstar::model {

std::filesystem::path settings_dir() {
    if (const char* dir = std::getenv("EINSTAR_SETTINGS_DIR"); dir && *dir) return dir;
    const char* home = std::getenv("HOME");
    return std::filesystem::path(home ? home : ".") / "Library" / "Application Support" / "Einstar";
}

namespace {
std::filesystem::path settings_file() { return settings_dir() / "model-settings.json"; }
}  // namespace

Settings load_settings() {
    Settings s;
    std::ifstream f(settings_file());
    if (!f) return s;
    try {
        nlohmann::json j = nlohmann::json::parse(f);
        s.tolerance_mm = j.value("tolerance_mm", s.tolerance_mm);
        s.deviation_range_mm = j.value("deviation_range_mm", s.deviation_range_mm);
        s.recent = j.value("recent", std::vector<std::string>{});
        for (const char* known : {"tolerance_mm", "deviation_range_mm", "recent"}) j.erase(known);
        s.other = std::move(j);
    } catch (const nlohmann::json::exception&) {
        // A damaged file: start from the defaults (it is rewritten on the next save).
    }
    return s;
}

Result<void> save_settings(const Settings& s) {
    std::error_code ec;
    std::filesystem::create_directories(settings_dir(), ec);
    nlohmann::json j = s.other;
    j["tolerance_mm"] = s.tolerance_mm;
    j["deviation_range_mm"] = s.deviation_range_mm;
    j["recent"] = s.recent;
    const auto tmp = settings_file().string() + ".tmp";
    {
        std::ofstream f(tmp);
        if (!f) return make_error(Errc::io, "cannot write " + tmp);
        f << j.dump(2) << '\n';
    }
    std::filesystem::rename(tmp, settings_file(), ec);
    if (ec) return make_error(Errc::io, "cannot write " + settings_file().string());
    return {};
}

void remember_recent(Settings& s, const std::filesystem::path& path) {
    const std::string p = std::filesystem::absolute(path).string();
    std::erase(s.recent, p);
    s.recent.insert(s.recent.begin(), p);
    if (s.recent.size() > 12) s.recent.resize(12);
}

}  // namespace einstar::model
