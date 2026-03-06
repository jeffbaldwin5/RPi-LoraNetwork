cmake_minimum_required(VERSION 3.18)
project(base_station_rpi CXX)

set(CMAKE_CXX_STANDARD 17)

# Use the local RadioLib copy (same version 7.6.0 as PlatformIO base station)
set(RADIOLIB_DIR "${CMAKE_SOURCE_DIR}/lib/RadioLib/src")

add_executable(base_station_rpi main.cpp)

target_include_directories(base_station_rpi PRIVATE ${RADIOLIB_DIR})

# Build RadioLib as a library from source
file(GLOB_RECURSE RADIOLIB_SOURCES "${RADIOLIB_DIR}/*.cpp")
add_library(radiolib_lib STATIC ${RADIOLIB_SOURCES})
target_include_directories(radiolib_lib PUBLIC ${RADIOLIB_DIR})

target_link_libraries(base_station_rpi PRIVATE radiolib_lib lgpio)
