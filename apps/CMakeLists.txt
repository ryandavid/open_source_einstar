if(EINSTAR_BUILD_APP)
add_executable(einstar-app MACOSX_BUNDLE
  einstar/main.mm
  einstar/app_state.cpp
  einstar/session.cpp
  einstar/workflow.mm
  einstar/agent_methods.cpp)
# A non-empty CFBundleIdentifier is required for the native file panels (NSOpenPanel): with an empty one the
# open/save panel XPC service never launches and runModal returns immediately with no dialog.
set_target_properties(einstar-app PROPERTIES OUTPUT_NAME Einstar
  MACOSX_BUNDLE_GUI_IDENTIFIER com.einstar.scan MACOSX_BUNDLE_BUNDLE_NAME Einstar)
target_include_directories(einstar-app PRIVATE einstar common)
target_link_libraries(einstar-app PRIVATE einstar::agent einstar::render einstar::pipeline einstar::recon einstar::session einstar::device einstar::sim einstar::calib einstar::synth imgui glfw TBB::tbb
  "-framework Cocoa" "-framework Metal" "-framework QuartzCore" "-framework ImageIO" "-framework CoreGraphics")
target_compile_options(einstar-app PRIVATE $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
einstar_target_defaults(einstar-app)

add_executable(einstar-calibrate MACOSX_BUNDLE
  einstar-calibrate/main.mm
  einstar-calibrate/calibration_controller.cpp
  einstar-calibrate/board_view.cpp
  einstar-calibrate/agent_methods.cpp)
set_target_properties(einstar-calibrate PROPERTIES OUTPUT_NAME EinstarCalibration
  MACOSX_BUNDLE_GUI_IDENTIFIER com.einstar.calibrate MACOSX_BUNDLE_BUNDLE_NAME EinstarCalibration)
target_include_directories(einstar-calibrate PRIVATE einstar-calibrate common)
target_compile_definitions(einstar-calibrate PRIVATE EINSTAR_CALIBRATION_FIXTURE="${PROJECT_SOURCE_DIR}/tests/fixtures/calibration/einstar_e10")
target_link_libraries(einstar-calibrate PRIVATE einstar::agent einstar::render_types einstar::calibrate einstar::device einstar::sim einstar::calib einstar::synth imgui glfw TBB::tbb
  "-framework Cocoa" "-framework Metal" "-framework QuartzCore" "-framework ImageIO" "-framework CoreGraphics")
target_compile_options(einstar-calibrate PRIVATE $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
einstar_target_defaults(einstar-calibrate)

# Scan to CAD (needs OpenCASCADE for solids and STEP: libs/brep, libs/model).
if(TARGET einstar::model)
add_executable(einstar-model MACOSX_BUNDLE
  einstar-model/main.mm
  einstar-model/model_app.cpp
  einstar-model/dialogs.mm
  einstar-model/photo_ui.mm
  einstar-model/agent_methods.cpp)
set_target_properties(einstar-model PROPERTIES OUTPUT_NAME EinstarModel
  MACOSX_BUNDLE_GUI_IDENTIFIER com.einstar.model MACOSX_BUNDLE_BUNDLE_NAME EinstarModel)
target_include_directories(einstar-model PRIVATE einstar-model common)
target_link_libraries(einstar-model PRIVATE einstar::agent einstar::render einstar::model einstar::image imgui glfw
  "-framework Cocoa" "-framework Metal" "-framework QuartzCore" "-framework ImageIO" "-framework CoreGraphics"
  "-framework UniformTypeIdentifiers")
target_compile_options(einstar-model PRIVATE $<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>)
einstar_target_defaults(einstar-model)
endif()
endif()

# Firmware: report the scanner's version, check an update package, flash one (docs/firmware.md 4).
add_executable(einstar-firmware einstar-firmware/main.cpp)
target_link_libraries(einstar-firmware PRIVATE einstar::device einstar::sim)
einstar_target_defaults(einstar-firmware)

add_executable(einstar-cli einstar-cli/main.cpp einstar-cli/replay.cpp)
target_link_libraries(einstar-cli PRIVATE einstar::device einstar::sim einstar::calib einstar::fixtures einstar::track einstar::track_metal einstar::session einstar::recon einstar::eval einstar::pipeline einstar::optim einstar::calibrate)
einstar_target_defaults(einstar-cli)
