# OpenCASCADE stays inside this library: its headers are system headers and nothing of it is public.
add_library(einstar_brep src/brep.cpp)
add_library(einstar::brep ALIAS einstar_brep)
target_include_directories(einstar_brep PUBLIC include)
target_include_directories(einstar_brep SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(einstar_brep PUBLIC einstar::core einstar::fit einstar::recon
                      PRIVATE TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool
                              TKShHealing TKFillet TKMesh TKDE TKXSBase TKDESTEP TKXCAF TKLCAF TKCAF)
einstar_target_defaults(einstar_brep)
