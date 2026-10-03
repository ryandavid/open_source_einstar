include(Catch)

# The Einstar's calibration (LeftCCF / RightCCF / TexCCF exactly as stored in the scanner's flash,
# dumped with `einstar-cli calib-dump`), so tests run the same on every machine.
add_compile_definitions(EINSTAR_TEST_CALIBRATION_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/calibration/einstar_e10")
# Excerpts of real EXStar data (einstar-cli fixture-pack; see fixtures/external/README.md and support/real_data.hpp).
add_compile_definitions(EINSTAR_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/external")
include_directories(support)

function(einstar_add_test name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} PRIVATE Catch2::Catch2WithMain)
  einstar_target_defaults(${name})
  catch_discover_tests(${name})
endfunction()

einstar_add_test(test_core core/test_se3.cpp core/test_spsc_ring.cpp core/test_lasso.cpp)
target_link_libraries(test_core PRIVATE einstar::core)

einstar_add_test(test_depth depth/test_stereo.cpp depth/test_metal_stereo.cpp)
target_link_libraries(test_depth PRIVATE einstar::depth_metal einstar::synth einstar::markers)

einstar_add_test(test_calib calib/test_rectify.cpp calib/test_device_calibration.cpp)
target_link_libraries(test_calib PRIVATE einstar::calib)

einstar_add_test(test_render render/test_scene_renderer.cpp render/test_view_camera.cpp render/test_scanner_model.cpp)
target_link_libraries(test_render PRIVATE einstar::render)

einstar_add_test(test_usb usb/test_codec.cpp)
target_link_libraries(test_usb PRIVATE einstar::usb)

einstar_add_test(test_device device/test_device_sim.cpp device/test_firmware.cpp)
target_link_libraries(test_device PRIVATE einstar::device einstar::sim)
target_compile_definitions(test_device PRIVATE EINSTAR_SOURCE_DIR="${PROJECT_SOURCE_DIR}" EINSTAR_BINARY_DIR="${PROJECT_BINARY_DIR}")

einstar_add_test(test_fixtures fixtures/test_packaging.cpp fixtures/test_exstar_project.cpp fixtures/test_fixture_tracking.cpp fixtures/test_global_registration.cpp fixtures/test_symmetry_probe.cpp)
target_link_libraries(test_fixtures PRIVATE einstar::fixtures einstar::track)

einstar_add_test(test_track track/test_tsdf.cpp track/test_metal_tsdf.cpp track/test_relocaliser.cpp)
target_link_libraries(test_track PRIVATE einstar::track einstar::track_metal)

einstar_add_test(test_pipeline pipeline/test_end_to_end.cpp pipeline/test_global_markers.cpp pipeline/test_marker_holes.cpp pipeline/test_recording.cpp)
target_link_libraries(test_pipeline PRIVATE einstar::pipeline einstar::device einstar::sim einstar::synth einstar::recon)
add_executable(debug_dump pipeline/debug_dump.cpp)
einstar_target_defaults(debug_dump)
target_link_libraries(debug_dump PRIVATE einstar::pipeline einstar::synth)
add_executable(debug_track pipeline/debug_track.cpp)
einstar_target_defaults(debug_track)
target_link_libraries(debug_track PRIVATE einstar::pipeline einstar::synth)

einstar_add_test(test_markers markers/test_markers.cpp)
target_link_libraries(test_markers PRIVATE einstar::markers einstar::calib einstar::fixtures)

einstar_add_test(test_calibrate calibrate/test_calibrate.cpp)
target_link_libraries(test_calibrate PRIVATE einstar::calibrate einstar::fixtures)

einstar_add_test(test_optim optim/test_marker_bundle.cpp)
target_link_libraries(test_optim PRIVATE einstar::optim)
add_executable(debug_markers pipeline/debug_markers.cpp)
einstar_target_defaults(debug_markers)
target_link_libraries(debug_markers PRIVATE einstar::pipeline einstar::synth)

einstar_add_test(test_session session/test_session.cpp)
target_link_libraries(test_session PRIVATE einstar::session)

einstar_add_test(test_recon recon/test_mesh.cpp recon/test_process.cpp)
target_link_libraries(test_recon PRIVATE einstar::recon einstar::fixtures einstar::synth)
add_executable(debug_depth_errors pipeline/debug_depth_errors.cpp)
einstar_target_defaults(debug_depth_errors)
target_link_libraries(debug_depth_errors PRIVATE einstar::pipeline einstar::synth)

einstar_add_test(test_fit fit/test_topology.cpp fit/test_primitive_fit.cpp fit/test_grow.cpp fit/test_holes_detect.cpp fit/test_solve.cpp)
target_link_libraries(test_fit PRIVATE einstar::fit)

if(EINSTAR_BUILD_APP)
  einstar_add_test(test_agent agent/test_agent.cpp)
  target_link_libraries(test_agent PRIVATE einstar::agent)
  # The MCP server (Python, tools/mcp) driving both apps headless with the emulator, when uv is installed.
  find_program(EINSTAR_UV uv)
  if(EINSTAR_UV)
    add_test(NAME einstar_mcp_e2e COMMAND ${EINSTAR_UV} run --directory ${PROJECT_SOURCE_DIR}/tools/mcp python tests/test_e2e.py)
    set_tests_properties(einstar_mcp_e2e PROPERTIES ENVIRONMENT "EINSTAR_BUILD_DIR=${PROJECT_BINARY_DIR}")
  endif()
endif()
# The MCP server's copy of the agent methods (tools/mcp/methods.json) matches the C++ table.
einstar_add_test(test_agent_methods agent/test_agent_methods.cpp)
target_link_libraries(test_agent_methods PRIVATE einstar::agent_protocol)
target_compile_definitions(test_agent_methods PRIVATE EINSTAR_METHODS_JSON="${PROJECT_SOURCE_DIR}/tools/mcp/methods.json")
