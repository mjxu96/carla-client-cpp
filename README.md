# Carla C++ Client

## Notice
Only being tested on Ubuntu 18.04

master branch is used for furthur development based on this c++ client and contains irrelevant codes. 

If you want to use client of 0.9.5/0.9.6 version, please checkout out to [0.9.5](https://github.com/wx9698/carla-client-cpp/tree/0.9.5)/[0.9.6](https://github.com/wx9698/carla-client-cpp/tree/0.9.6) branch.

## Instruction
This repo is used to generate carla c++ client with least source files. 

It will also try to use GNU build tools (like g++ instead of clang++) to build this client. For fast build, this repo also retain the way to build with clang (clang is not included in following setup).

## Usage:
```
$ git clone https://github.com/wx9698/carla-client-cpp.git
$ cd carla-client-cpp

# install necessary carla build tools
$ sudo ./setup/install-carla-tool.sh

# download boost, rpc, gtest and recast lib to this client folder
$ ./setup/setup.sh                               
# you can use command "./setup/setup.sh clang" to indicate script to build with clang

# build
$ mkdir build && cd build
$ cmake ../
# you can also use command "cmake -DUSE_CLANG=ON ../" to indicate cmake to set default compiler to clang
$ make main                                      # it will generate the binary program at REPO_ROOT_FOLDER/bin
$ ../bin/main
```
