cmake_minimum_required(VERSION 3.20)
project(nosim
  VERSION 0.1.0
  DESCRIPTION "Engine-independent core for the planetary-to-translunar world simulator"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Header-only library: everything lives under include/nosim and is double-precision,
# allocation-free math suitable for both the headless world-compiler and the UE5 client.
add_library(nosim INTERFACE)
add_library(nosim::nosim ALIAS nosim)
target_include_directories(nosim INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(nosim INTERFACE cxx_std_20)

option(NOSIM_BUILD_TESTS "Build the nosim verification tests" ON)
if(NOSIM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
