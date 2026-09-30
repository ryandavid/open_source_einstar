if(EINSTAR_BUILD_APP)
add_executable(einstar-app MACOSX_BUNDLE
  einstar/main.mm
  einstar/app_state.cpp
  einstar/session.cpp)
set_target_properties(einstar-app PROPERTIES OUTPUT_NAME Einstar)
target_include_directories(einstar-app PRIVATE einstar)
target_link_libraries(einstar-app PRIVATE einstar::render einstar::pipeline einstar::recon einstar::session einstar::device einstar::sim einstar::calib einstar::calibrate einstar::synth imgui glfw TBB::tbb
  "-framework Cocoa" "-framework Metal" "-framework QuartzCore" "-framework ImageIO" "-framework CoreGraphics")
target_compile_options(einstar-app PRIVATE $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
einstar_target_defaults(einstar-app)

add_executable(einstar-calibrate MACOSX_BUNDLE
  einstar-calibrate/main.mm
  einstar-calibrate/calibration_controller.cpp)
set_target_properties(einstar-calibrate PROPERTIES OUTPUT_NAME EinstarCalibration)
target_include_directories(einstar-calibrate PRIVATE einstar-calibrate)
target_compile_definitions(einstar-calibrate PRIVATE EINSTAR_CALIBRATION_FIXTURE="${PROJECT_SOURCE_DIR}/tests/fixtures/calibration/einstar_e10")
target_link_libraries(einstar-calibrate PRIVATE einstar::calibrate einstar::device einstar::sim einstar::calib einstar::synth imgui glfw TBB::tbb
  "-framework Cocoa" "-framework Metal" "-framework QuartzCore" "-framework ImageIO" "-framework CoreGraphics")
target_compile_options(einstar-calibrate PRIVATE $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
einstar_target_defaults(einstar-calibrate)
endif()

add_executable(einstar-cli einstar-cli/main.cpp)
target_link_libraries(einstar-cli PRIVATE einstar::device einstar::sim einstar::calib einstar::fixtures einstar::track einstar::track_metal einstar::session einstar::recon einstar::eval einstar::pipeline einstar::optim einstar::calibrate)
einstar_target_defaults(einstar-cli)
