cmake_minimum_required(VERSION 3.5.1)

SET(CMAKE_C_COMPILER /usr/bin/clang)
SET(CMAKE_CXX_COMPILER /usr/bin/clang++)

find_library(libcarla_client NAMES carla_client PATHS ../../lib)
find_library(librpc NAMES rpc PATHS ../../lib)
find_library(libboost_filesystem NAMES boost_filesystem PATHS ../../lib)

find_package(PNG)
find_package(TIFF)
find_package(JPEG)

add_executable(main main.cpp)

include_directories(../../include/lib)
include_directories(${PNG_INCLUDE_DIRS})
include_directories(${TIFF_INCLUDE_DIRS})
include_directories(${JPEG_INCLUDE_DIRS})

target_link_libraries(main libcarla_client librpc libboost_filesystem ${PNG_LIBRARIES} ${TIFF_LIBRARIES} ${JPEG_LIBRARIES})