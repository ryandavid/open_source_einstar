#include "einstar/agent/protocol.hpp"

namespace einstar::agent {

std::string request_line(std::int64_t id, std::string_view method, const json& params) {
    const json j = {{"jsonrpc", "2.0"}, {"id", id}, {"method", method}, {"params", params.is_null() ? json::object() : params}};
    return j.dump() + "\n";  // dump() escapes newlines: one line per message
}

std::string response_line(const json& id, const Result& result) {
    json j = {{"jsonrpc", "2.0"}, {"id", id}};
    if (const auto* r = std::get_if<json>(&result)) {
        j["result"] = *r;
    } else {
        const auto& e = std::get<Error>(result);
        j["error"] = {{"code", static_cast<int>(e.code)}, {"message", e.message}};
        if (!e.data.is_null()) j["error"]["data"] = e.data;
    }
    // Invalid UTF-8 from anywhere (a log line, a file name) must not make the reply undeliverable.
    return j.dump(-1, ' ', false, json::error_handler_t::replace) + "\n";
}

Result parse_response(std::string_view line) {
    const json j = json::parse(line, nullptr, false);
    if (j.is_discarded() || !j.is_object()) return error(ErrorCode::internal, "malformed response from the app");
    if (const auto it = j.find("error"); it != j.end() && it->is_object()) {
        Error e;
        e.code = static_cast<ErrorCode>(it->value("code", static_cast<int>(ErrorCode::internal)));
        e.message = it->value("message", std::string("(no message)"));
        if (const auto d = it->find("data"); d != it->end()) e.data = *d;
        return e;
    }
    if (const auto it = j.find("result"); it != j.end()) return *it;
    return error(ErrorCode::internal, "response has neither result nor error");
}

}  // namespace einstar::agent
