#pragma once

#include <atomic>
#include <format>
#include <functional>
#include <string_view>

namespace einstar::log {

enum class Level : int { trace = 0, debug, info, warn, error, off };

void set_level(Level level);
[[nodiscard]] Level level();
void write(Level level, std::string_view message);
// Also hand every record written to `sink` (besides stderr), e.g. an agent's log ring; empty removes it.
void set_sink(std::function<void(Level, std::string_view)> sink);
[[nodiscard]] std::string_view level_name(Level level);  // trace, debug, info, warn, error

template <typename... Args>
void emit(Level lvl, std::format_string<Args...> fmt, Args&&... args) {
    if (lvl < level()) return;
    write(lvl, std::format(fmt, std::forward<Args>(args)...));
}

template <typename... Args>
void trace(std::format_string<Args...> fmt, Args&&... args) { emit(Level::trace, fmt, std::forward<Args>(args)...); }
template <typename... Args>
void debug(std::format_string<Args...> fmt, Args&&... args) { emit(Level::debug, fmt, std::forward<Args>(args)...); }
template <typename... Args>
void info(std::format_string<Args...> fmt, Args&&... args) { emit(Level::info, fmt, std::forward<Args>(args)...); }
template <typename... Args>
void warn(std::format_string<Args...> fmt, Args&&... args) { emit(Level::warn, fmt, std::forward<Args>(args)...); }
template <typename... Args>
void error(std::format_string<Args...> fmt, Args&&... args) { emit(Level::error, fmt, std::forward<Args>(args)...); }

}  // namespace einstar::log
