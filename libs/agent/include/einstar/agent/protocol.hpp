#pragma once

// Agent control: what an app embedding the agent server (agent/server.hpp) can be asked to do, and how.
//
//   Claude Code --MCP (stdio)--> einstar-mcp --JSON-RPC 2.0, one JSON object per line, AF_UNIX--> app --mcp=<socket>
//
// Every method is described once, here (method_specs()): its name, the app that answers it, whether it
// changes state, a description and a JSON schema of its parameters. The apps register handlers against
// these specs and einstar-mcp turns the same table into MCP tools, so the two cannot drift apart.
//
// Coordinates are logical window points (ImGui's), origin at the window's top left; never screen or
// framebuffer pixels. A screenshot reports its scale, so an image pixel maps back to a point.

#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

namespace einstar::agent {

using json = nlohmann::json;

// Which app answers a method. `any`: the generic layer every app has (ui.*, input.*, app.*).
enum class App { any, scan, calibration, model };
[[nodiscard]] std::string_view app_name(App a);  // "any", "scan", "calibration", "model"
[[nodiscard]] std::optional<App> app_from_name(std::string_view name);

// Whether a method changes state. A mutating method answers only after the app has drawn two more
// frames, so a screenshot right after it shows its effect.
enum class Kind { read_only, mutating };

struct MethodSpec {
    std::string name;  // dotted, namespaced by area: "ui.snapshot", "scan.start"
    App app = App::any;
    Kind kind = Kind::read_only;
    std::string description;
    json params;  // JSON schema (an object schema)
};

[[nodiscard]] const std::vector<MethodSpec>& method_specs();
[[nodiscard]] const MethodSpec* find_spec(std::string_view name);
// The table as JSON: [{name, app, kind, description, params}, ...]. Committed as tools/mcp/methods.json, which
// the MCP server (Python) reads; a test keeps the two equal (einstar-agent-methods regenerates it).
[[nodiscard]] json method_specs_json();

// JSON-RPC errors. The standard codes, and the app's own answers.
enum class ErrorCode : int {
    parse = -32700,
    invalid_request = -32600,
    method_not_found = -32601,
    invalid_params = -32602,
    internal = -32603,
    busy = -32000,       // the UI thread did not get to it in time (a stuck frame loop): an answer, not a hang
    not_found = -32001,  // a selector matched nothing
    ambiguous = -32002,  // a selector matched more than one item (never a silent first match)
    refused = -32003,    // the app cannot do it now (not connected, already scanning, ...)
    timeout = -32004,    // a wait ran out
};

struct Error {
    ErrorCode code = ErrorCode::internal;
    std::string message;
    json data;  // optional detail (e.g. the candidates of an ambiguous selector)
};

using Result = std::variant<json, Error>;

[[nodiscard]] inline Error error(ErrorCode code, std::string message, json data = nullptr) {
    return {code, std::move(message), std::move(data)};
}

// Framing: one JSON object per line (no raw newline inside; JSON escapes them).
[[nodiscard]] std::string request_line(std::int64_t id, std::string_view method, const json& params);
[[nodiscard]] std::string response_line(const json& id, const Result& result);
// A response line back into a result (a malformed line is an internal error).
[[nodiscard]] Result parse_response(std::string_view line);

}  // namespace einstar::agent
