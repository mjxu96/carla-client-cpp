
cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CLIENT_ROOT_FOLDER}/bin)

include_directories(${CLIENT_ROOT_FOLDER}/include/lib)
include_directories(${CLIENT_ROOT_FOLDER}/include)
link_directories(${CLIENT_ROOT_FOLDER}/lib)

add_library(router-lib common.cpp)
add_library(router router.cpp)
add_executable(router-test router.cpp)

target_link_libraries(router router-lib common carla_client rpc boost_filesystem png tiff jpeg)
target_link_libraries(router-test router)
target_include_directories(router PUBLIC .)