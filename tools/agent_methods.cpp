// Prints the agent methods (agent::method_specs()) as JSON, for tools/mcp/methods.json:
//   build/tools/einstar-agent-methods > tools/mcp/methods.json

#include <iostream>

#include "einstar/agent/protocol.hpp"

int main() {
    std::cout << einstar::agent::method_specs_json().dump(2) << "\n";
    return 0;
}
