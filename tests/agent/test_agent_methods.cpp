// tools/mcp/methods.json -- what the MCP server lists as tools -- is the C++ method table, exactly.

#include <catch2/catch_test_macros.hpp>

#include <fstream>
#include <sstream>

#include "einstar/agent/protocol.hpp"

TEST_CASE("tools/mcp/methods.json matches the agent methods") {
    std::ifstream in(EINSTAR_METHODS_JSON);
    REQUIRE(in.good());
    std::stringstream ss;
    ss << in.rdbuf();
    const auto committed = einstar::agent::json::parse(ss.str(), nullptr, false);
    REQUIRE_FALSE(committed.is_discarded());
    INFO("stale: regenerate with  build/tools/einstar-agent-methods > tools/mcp/methods.json");
    CHECK(committed == einstar::agent::method_specs_json());
}
