cmake_minimum_required(VERSION 3.5.1)

find_library(CARLA_CLIENT_LIB carla_client PATHS ../../lib)
find_library(RPC_LIB rpc PATHS ../../lib)
find_library(BOOST_FILESYSTEM_LIB boost_filesystem PATHS ../../lib)

find_library(PNG_LIB png)
find_library(TIFF_LIB tiff)
find_library(JPEG_LIB jpeg)

add_executable(main main.cpp)

target_link_libraries(main CARLA_CLIENT_LIB RPC_LIB BOOST_FILESYSTEM_LIB PNG_LIB TIFF_LIB JPEG_LIB)
target_include_directories(main PUBLIC ../../include)