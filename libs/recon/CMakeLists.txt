add_library(einstar_recon src/extract.cpp src/mesh_ops.cpp src/simplify.cpp src/registration.cpp src/consistency.cpp src/process.cpp
                          src/watertight.cpp)
add_library(einstar::recon ALIAS einstar_recon)
target_include_directories(einstar_recon PUBLIC include)
target_link_libraries(einstar_recon PUBLIC einstar::core einstar::track einstar::session
                      PRIVATE einstar::optim einstar::track_metal einstar::gpu nanoflann_headers TBB::tbb einstar_poisson)
einstar_target_defaults(einstar_recon)

# PoissonRecon (third-party, MIT) needs C++20 (std::is_pod_v is gone from C++23's library): built on
# its own behind a plain interface (src/poisson.hpp), without our warning flags.
add_library(einstar_poisson STATIC src/poisson.cpp)
set_target_properties(einstar_poisson PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
target_link_libraries(einstar_poisson PRIVATE poissonrecon_headers)
target_compile_options(einstar_poisson PRIVATE -Wno-deprecated-declarations $<$<CONFIG:Release,RelWithDebInfo>:-O3>)
