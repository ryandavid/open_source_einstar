#include "einstar/agent/client.hpp"

#include <chrono>
#include <cstring>
#include <format>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace einstar::agent {

std::variant<std::unique_ptr<Client>, std::string> Client::connect(const std::string& socket_path) {
    sockaddr_un addr{};
    if (socket_path.size() >= sizeof(addr.sun_path)) return std::string("socket path too long: ") + socket_path;
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return std::format("socket(): {}", std::strerror(errno));
    addr.sun_family = AF_UNIX;
    std::memcpy(addr.sun_path, socket_path.c_str(), socket_path.size() + 1);
    if (::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        const std::string why = std::format("connect({}): {}", socket_path, std::strerror(errno));
        ::close(fd);
        return why;
    }
    return std::unique_ptr<Client>(new Client(fd));
}

Client::~Client() {
    if (fd_ >= 0) ::close(fd_);
}

Result Client::call(std::string_view method, const json& params, int timeout_ms) {
    const std::int64_t id = next_id_++;
    const std::string line = request_line(id, method, params);
    for (std::size_t sent = 0; sent < line.size();) {
        const ssize_t n = ::send(fd_, line.data() + sent, line.size() - sent, 0);
        if (n <= 0) return error(ErrorCode::internal, std::format("send: {}", n < 0 ? std::strerror(errno) : "connection closed"));
        sent += static_cast<std::size_t>(n);
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (;;) {
        // Every complete line: the answer to this call, or a late one to skip.
        for (std::size_t nl; (nl = buffer_.find('\n')) != std::string::npos;) {
            const std::string reply = buffer_.substr(0, nl);
            buffer_.erase(0, nl + 1);
            const json j = json::parse(reply, nullptr, false);
            if (!j.is_discarded() && j.is_object() && j.value("id", json()) == json(id)) return parse_response(reply);
        }
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
        if (left <= 0) return error(ErrorCode::timeout, std::format("{}: no reply within {} ms", method, timeout_ms));
        pollfd p{fd_, POLLIN, 0};
        const int r = ::poll(&p, 1, static_cast<int>(left));
        if (r < 0 && errno != EINTR) return error(ErrorCode::internal, std::format("poll: {}", std::strerror(errno)));
        if (r <= 0) continue;
        char chunk[65536];
        const ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);
        if (n <= 0) return error(ErrorCode::internal, "the app closed the connection");
        buffer_.append(chunk, static_cast<std::size_t>(n));
    }
}

}  // namespace einstar::agent
