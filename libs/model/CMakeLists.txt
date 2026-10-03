# The modelling document and its commands (needs libs/brep, i.e. OpenCASCADE).
add_library(einstar_model src/document.cpp src/commands.cpp src/json.cpp src/io.cpp src/settings.cpp src/photo.cpp src/photo_render.cpp src/photo_geometry.cpp)
add_library(einstar::model ALIAS einstar_model)
target_include_directories(einstar_model PUBLIC include)
target_link_libraries(einstar_model PUBLIC einstar::fit einstar::brep einstar::recon nlohmann_json::nlohmann_json
                      PRIVATE einstar::session einstar::image einstar_zstd TBB::tbb)
einstar_target_defaults(einstar_model)
