cmake_minimum_required(VERSION 3.28)
project(einstar VERSION 0.1.0 LANGUAGES C CXX OBJC OBJCXX)

get_property(EINSTAR_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT EINSTAR_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified; defaulting to RelWithDebInfo")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_OBJCXX_STANDARD 23)

# Older Apple libc++ (e.g. CLT 16 / Apple clang 17) keeps std::jthread and
# std::stop_token behind -fexperimental-library.
include(CheckCXXSourceCompiles)
set(EINSTAR_JTHREAD_TEST "#include <stop_token>\n#include <thread>\nint main(){std::jthread t([](std::stop_token){});}")
set(CMAKE_REQUIRED_FLAGS "-std=c++23")
check_cxx_source_compiles("${EINSTAR_JTHREAD_TEST}" EINSTAR_HAVE_JTHREAD)
if(NOT EINSTAR_HAVE_JTHREAD)
  set(CMAKE_REQUIRED_FLAGS "-std=c++23 -fexperimental-library")
  check_cxx_source_compiles("${EINSTAR_JTHREAD_TEST}" EINSTAR_HAVE_JTHREAD_EXPERIMENTAL)
  if(NOT EINSTAR_HAVE_JTHREAD_EXPERIMENTAL)
    message(FATAL_ERROR "The C++ standard library lacks std::jthread; update Xcode / the Command Line Tools")
  endif()
  add_compile_options($<$<COMPILE_LANGUAGE:CXX,OBJCXX>:-fexperimental-library>)
  add_link_options(-fexperimental-library)
endif()
unset(CMAKE_REQUIRED_FLAGS)

option(EINSTAR_BUILD_TESTS "Build unit tests" ON)
option(EINSTAR_BUILD_APP "Build the GUI application (GLFW + Dear ImGui + Metal)" ON)
option(EINSTAR_WITH_RERUN "Enable Rerun debug visualisation" OFF)
option(EINSTAR_BUILD_FIRMWARE "Build the scanner's FX3 firmware (needs the Arm GNU Toolchain and the user-supplied FX3 SDK, see firmware/README.md)" OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(EinstarWarnings)
include(Dependencies)

add_subdirectory(libs)
add_subdirectory(tools)
add_subdirectory(apps)

if(EINSTAR_BUILD_FIRMWARE)
  include(Firmware)
endif()

if(EINSTAR_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
