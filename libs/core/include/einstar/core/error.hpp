#pragma once

#include <expected>
#include <string>

namespace einstar {

enum class Errc {
    ok = 0,
    not_found,
    io,
    timeout,
    protocol,
    invalid_argument,
    blocked,  // refused by a safety guard
    unsupported,
    busy,
    disconnected,
};

struct Error {
    Errc code = Errc::ok;
    std::string message;
};

template <typename T>
using Result = std::expected<T, Error>;

[[nodiscard]] inline std::unexpected<Error> make_error(Errc code, std::string message) {
    return std::unexpected(Error{code, std::move(message)});
}

}  // namespace einstar
