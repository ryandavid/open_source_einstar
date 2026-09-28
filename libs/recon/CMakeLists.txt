add_library(einstar_recon src/extract.cpp src/mesh_ops.cpp src/simplify.cpp src/registration.cpp src/process.cpp)
add_library(einstar::recon ALIAS einstar_recon)
target_include_directories(einstar_recon PUBLIC include)
target_link_libraries(einstar_recon PUBLIC einstar::core einstar::track einstar::session
                      PRIVATE einstar::optim einstar::track_metal einstar::gpu nanoflann_headers TBB::tbb)
einstar_target_defaults(einstar_recon)
