cmake_minimum_required(VERSION 3.0.2)
project(kdmp)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(omplRRT execs/omplRRT.cpp)