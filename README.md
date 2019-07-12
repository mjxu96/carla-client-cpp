# Carla C++ Client for 095 release version

## Notice 
Only tested on Ubuntu 18.04 LTS

Only compatible with [Carla 0.9.5 Release Version](https://github.com/carla-simulator/carla/releases/tag/0.9.5)

This repo is used to generate carla c++ client with least source files. 

It will also try to use GNU build tools (like g++ instead of clang++) to build this client. For fast build, this repo also retain the way to build with clang (clang is not included in following setup).

## Usage:
```
$ git clone https://github.com/wx9698/carla-client-cpp.git
$ cd carla-client-cpp
$ sudo ./setup/install-carla-tool.sh             # install necessary carla build tools
$ ./setup/setup.sh                               
# you can use command ./setup/setup.sh clang to indicate script to build with clang
$ mkdir build && cd build
$ cmake ../
# you can also use command cmake -DUSE_CLANG=ON ../ to indicate cmake to set default compiler to clang
$ make main                                      # it will generate the binary program at REPO_ROOT_FOLDER/bin
$ ../bin/main
```
