add_executable(einstar-bench bench/bench.cpp)
target_link_libraries(einstar-bench PRIVATE einstar::pipeline einstar::synth einstar::fixtures einstar::track)
einstar_target_defaults(einstar-bench)
