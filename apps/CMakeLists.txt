if(EINSTAR_BUILD_APP)
add_executable(einstar-app MACOSX_BUNDLE
  einstar/main.mm
  einstar/app_state.cpp
  einstar/session.cpp)
set_target_properties(einstar-app PROPERTIES OUTPUT_NAME Einstar)
target_include_directories(einstar-app PRIVATE einstar)
target_link_libraries(einstar-app PRIVATE einstar::render einstar::pipeline einstar::recon einstar::session einstar::device einstar::sim einstar::calib einstar::synth imgui glfw TBB::tbb
  "-framework Cocoa" "-framework Metal" "-framework QuartzCore" "-framework ImageIO" "-framework CoreGraphics")
target_compile_options(einstar-app PRIVATE $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
einstar_target_defaults(einstar-app)
endif()

add_executable(einstar-cli einstar-cli/main.cpp)
target_link_libraries(einstar-cli PRIVATE einstar::device einstar::sim einstar::calib einstar::fixtures einstar::track einstar::track_metal einstar::session einstar::recon einstar::eval einstar::pipeline)
einstar_target_defaults(einstar-cli)
