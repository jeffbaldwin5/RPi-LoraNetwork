cmake_minimum_required(VERSION 3.18)
project(base_station_rpi CXX)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
    RadioLib
    GIT_REPOSITORY https://github.com/jgromes/RadioLib.git
    GIT_TAG        master
)
# Only download sources — don't process RadioLib's own CMakeLists
# (it assumes Arduino by default)
FetchContent_GetProperties(RadioLib)
if(NOT radiolib_POPULATED)
    FetchContent_Populate(RadioLib)
endif()

add_executable(base_station_rpi main.cpp)

target_include_directories(base_station_rpi PRIVATE
    ${radiolib_SOURCE_DIR}/src
)

# Build RadioLib as a library from source
file(GLOB_RECURSE RADIOLIB_SOURCES "${radiolib_SOURCE_DIR}/src/*.cpp")
add_library(radiolib_lib STATIC ${RADIOLIB_SOURCES})
target_include_directories(radiolib_lib PUBLIC ${radiolib_SOURCE_DIR}/src)
target_compile_definitions(radiolib_lib PUBLIC RADIOLIB_BUILD_ARDUINO=0)

target_link_libraries(base_station_rpi PRIVATE radiolib_lib lgpio)
