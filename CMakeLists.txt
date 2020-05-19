cmake_minimum_required(VERSION 3.10.0)

# Setting up project variables.
if(NOT DEFINED WOP_VERSION_MAJOR)
  set(WOP_VERSION_MAJOR 0)
endif()
if(NOT DEFINED WOP_VERSION_MINOR)
  set(WOP_VERSION_MINOR 0)
endif()
if(NOT DEFINED WOP_VERSION_PATCH)
  set(WOP_VERSION_PATCH 0)
endif()

project(WOP
  VERSION ${WOP_VERSION_MAJOR}.${WOP_VERSION_MINOR}.${WOP_VERSION_PATCH}
  LANGUAGES CXX)

# CMake build variables.

set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ standard to conform to")
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "No build type selected, default to Debug")
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type (default Debug)" FORCE)
endif()

# Directories and sources.

set(WOP_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(WOP_DATA_DIR ${CMAKE_SOURCE_DIR}/data)
set(WOP_TEST_DIR ${CMAKE_SOURCE_DIR}/test)

# add_subdirectory(lib)
add_subdirectory(tools)

configure_file(WOPConfig.h.in WOPConfig.h)
