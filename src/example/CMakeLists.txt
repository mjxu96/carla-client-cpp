cmake_minimum_required(VERSION 3.5.1)

SET(CMAKE_C_COMPILER /usr/bin/clang)
SET(CMAKE_CXX_COMPILER /usr/bin/clang++)

find_library(carla_client PATHS ../../lib)
find_library(rpc PATHS ../../lib)
find_library(boost_filesystem PATHS ../../lib)

find_library(png)
find_library(tiff)
find_library(jpeg)

add_executable(main main.cpp)

target_link_libraries(main carla_client rpc boost_filesystem png tiff jpeg)
target_include_directories(main PUBLIC ../../include/lib)