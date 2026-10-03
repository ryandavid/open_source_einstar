#pragma once

// A connection to an app's agent socket (einstar-mcp, tests).

#include <memory>
#include <string>
#include <variant>

#include "einstar/agent/protocol.hpp"

namespace einstar::agent {

class Client {
public:
    // The client, or why it could not connect.
    static std::variant<std::unique_ptr<Client>, std::string> connect(const std::string& socket_path);
    ~Client();
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    // One request and its response. A reply that arrives after its call timed out is skipped when the
    // next call reads (responses are matched by id).
    Result call(std::string_view method, const json& params = json::object(), int timeout_ms = 10000);

private:
    explicit Client(int fd) : fd_(fd) {}
    int fd_ = -1;
    std::int64_t next_id_ = 1;
    std::string buffer_;  // bytes read past the last line
};

}  // namespace einstar::agent
