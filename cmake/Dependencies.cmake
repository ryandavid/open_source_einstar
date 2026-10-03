# System packages (Homebrew on macOS) and fetched header/source deps.
include(FetchContent)
set(FETCHCONTENT_QUIET ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
find_package(TBB REQUIRED)
# Homebrew keeps TBB's headers in a directory the compiler searches implicitly
# (/usr/local/include on Intel, /opt/homebrew/include on Apple Silicon). CMake
# drops implicit directories from the compile line, so the -isystem that would
# mark TBB::tbb's (imported, hence system) headers as system headers is never
# emitted -- and TBB's own headers then get compiled under our -Werror warning
# flags, e.g. -Wsign-conversion firing inside <tbb/blocked_range.h>. Drop TBB's
# include dir from the implicit list so it is passed as -isystem, like every
# other third-party dependency here. (Harmless if TBB lives off a non-implicit
# path: the dir is simply not in the list to remove.)
get_target_property(_einstar_tbb_includes TBB::tbb INTERFACE_INCLUDE_DIRECTORIES)
if(_einstar_tbb_includes)
  list(REMOVE_ITEM CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES ${_einstar_tbb_includes})
  list(REMOVE_ITEM CMAKE_OBJCXX_IMPLICIT_INCLUDE_DIRECTORIES ${_einstar_tbb_includes})
endif()
unset(_einstar_tbb_includes)
find_package(zstd CONFIG QUIET)
if(NOT zstd_FOUND)
  pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)
  add_library(einstar_zstd INTERFACE)
  target_link_libraries(einstar_zstd INTERFACE PkgConfig::ZSTD)
else()
  add_library(einstar_zstd INTERFACE)
  target_link_libraries(einstar_zstd INTERFACE $<IF:$<TARGET_EXISTS:zstd::libzstd_shared>,zstd::libzstd_shared,zstd::libzstd_static>)
endif()
find_package(nlohmann_json REQUIRED)
find_package(Ceres QUIET)

# nanoflann: header-only k-d trees (BSD).
FetchContent_Declare(nanoflann
  URL https://github.com/jlblancoc/nanoflann/archive/refs/tags/v1.7.1.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP ON)
set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nanoflann)
add_library(nanoflann_headers INTERFACE)
target_include_directories(nanoflann_headers SYSTEM INTERFACE ${nanoflann_SOURCE_DIR}/include)

# metal-cpp: header-only C++ bindings for Metal.
FetchContent_Declare(metal_cpp
  URL https://developer.apple.com/metal/cpp/files/metal-cpp_macOS15_iOS18.zip
  DOWNLOAD_EXTRACT_TIMESTAMP ON)
FetchContent_MakeAvailable(metal_cpp)
add_library(metal_cpp INTERFACE)
target_include_directories(metal_cpp SYSTEM INTERFACE ${metal_cpp_SOURCE_DIR})
target_link_libraries(metal_cpp INTERFACE
  "-framework Metal" "-framework Foundation" "-framework QuartzCore")

if(EINSTAR_BUILD_TESTS)
  FetchContent_Declare(Catch2
    URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.8.1.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    SYSTEM)
  FetchContent_MakeAvailable(Catch2)
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()

if(EINSTAR_BUILD_APP)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(glfw
    URL https://github.com/glfw/glfw/archive/refs/tags/3.4.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    SYSTEM)
  FetchContent_MakeAvailable(glfw)

  FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.92.1.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP ON)
  FetchContent_MakeAvailable(imgui)
  add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_metal.mm)
  target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
  target_link_libraries(imgui PUBLIC glfw "-framework Metal" "-framework QuartzCore" "-framework Cocoa")
  target_compile_options(imgui PRIVATE -fobjc-arc -w)
  # Items are reported to hooks as they are drawn (the agent's UI inspector, libs/agent); public, so every
  # file including ImGui's headers sees the same context layout.
  target_compile_definitions(imgui PUBLIC IMGUI_ENABLE_TEST_ENGINE)
endif()
