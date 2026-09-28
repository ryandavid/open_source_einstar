include(Catch)

function(einstar_add_test name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} PRIVATE Catch2::Catch2WithMain)
  einstar_target_defaults(${name})
  catch_discover_tests(${name})
endfunction()

einstar_add_test(test_core core/test_se3.cpp core/test_spsc_ring.cpp)
target_link_libraries(test_core PRIVATE einstar::core)

einstar_add_test(test_depth depth/test_stereo.cpp depth/test_metal_stereo.cpp)
target_link_libraries(test_depth PRIVATE einstar::depth_metal einstar::synth)

einstar_add_test(test_calib calib/test_rectify.cpp calib/test_device_calibration.cpp)
target_link_libraries(test_calib PRIVATE einstar::calib)

einstar_add_test(test_render render/test_scene_renderer.cpp)
target_link_libraries(test_render PRIVATE einstar::render)

einstar_add_test(test_usb usb/test_codec.cpp)
target_link_libraries(test_usb PRIVATE einstar::usb)

einstar_add_test(test_device device/test_device_sim.cpp)
target_link_libraries(test_device PRIVATE einstar::device einstar::sim)

einstar_add_test(test_fixtures fixtures/test_exstar_project.cpp fixtures/test_fixture_tracking.cpp fixtures/test_global_registration.cpp fixtures/test_symmetry_probe.cpp)
target_link_libraries(test_fixtures PRIVATE einstar::fixtures einstar::track)

einstar_add_test(test_track track/test_tsdf.cpp track/test_metal_tsdf.cpp)
target_link_libraries(test_track PRIVATE einstar::track einstar::track_metal)

einstar_add_test(test_pipeline pipeline/test_end_to_end.cpp pipeline/test_global_markers.cpp pipeline/test_marker_holes.cpp)
target_link_libraries(test_pipeline PRIVATE einstar::pipeline einstar::device einstar::sim einstar::synth einstar::recon)
add_executable(debug_dump pipeline/debug_dump.cpp)
target_link_libraries(debug_dump PRIVATE einstar::pipeline einstar::synth)
add_executable(debug_track pipeline/debug_track.cpp)
target_link_libraries(debug_track PRIVATE einstar::pipeline einstar::synth)

einstar_add_test(test_markers markers/test_markers.cpp)
target_link_libraries(test_markers PRIVATE einstar::markers einstar::calib)

einstar_add_test(test_optim optim/test_marker_bundle.cpp)
target_link_libraries(test_optim PRIVATE einstar::optim)
add_executable(debug_markers pipeline/debug_markers.cpp)
target_link_libraries(debug_markers PRIVATE einstar::pipeline einstar::synth)

einstar_add_test(test_session session/test_session.cpp)
target_link_libraries(test_session PRIVATE einstar::session)

einstar_add_test(test_recon recon/test_mesh.cpp recon/test_process.cpp)
target_link_libraries(test_recon PRIVATE einstar::recon einstar::fixtures einstar::synth)
