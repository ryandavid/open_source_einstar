# Agent control (protocol.hpp). The protocol -- method specs, framing, the socket client -- is shared by
# the apps and einstar-mcp; the server (with ImGui inspection and input) is embedded in the GUI apps.
add_library(einstar_agent_protocol src/protocol.cpp src/specs.cpp src/client.cpp)
add_library(einstar::agent_protocol ALIAS einstar_agent_protocol)
target_include_directories(einstar_agent_protocol PUBLIC include)
target_link_libraries(einstar_agent_protocol PUBLIC nlohmann_json::nlohmann_json)
einstar_target_defaults(einstar_agent_protocol)

if(EINSTAR_BUILD_APP)
  add_library(einstar_agent src/server.cpp src/ui.cpp src/log_ring.cpp src/image.cpp)
  add_library(einstar::agent ALIAS einstar_agent)
  target_include_directories(einstar_agent PUBLIC include)
  target_link_libraries(einstar_agent PUBLIC einstar::agent_protocol einstar::core imgui
                        PRIVATE "-framework CoreGraphics" "-framework ImageIO" "-framework CoreFoundation")
  einstar_target_defaults(einstar_agent)
endif()
