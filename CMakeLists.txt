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
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(EINSTAR_BUILD_TESTS "Build unit tests" ON)
option(EINSTAR_BUILD_APP "Build the GUI application (GLFW + Dear ImGui + Metal)" ON)
option(EINSTAR_WITH_RERUN "Enable Rerun debug visualisation" OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(EinstarWarnings)
include(Dependencies)

add_subdirectory(libs)
add_subdirectory(tools)
add_subdirectory(apps)

if(EINSTAR_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
