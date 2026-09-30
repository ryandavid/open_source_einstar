add_executable(einstar-bench bench/bench.cpp)
target_link_libraries(einstar-bench PRIVATE einstar::pipeline einstar::synth einstar::fixtures einstar::track einstar::track_metal)
einstar_target_defaults(einstar-bench)
# The scanner's calibration (see tests/CMakeLists.txt), used when EXStar's cache is not installed.
target_compile_definitions(einstar-bench PRIVATE EINSTAR_CALIBRATION_FIXTURE="${PROJECT_SOURCE_DIR}/tests/fixtures/calibration/einstar_e10")
