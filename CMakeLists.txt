cmake_minimum_required(VERSION 3.5)
project(kinodynamic_motion_planning VERSION 0.01 LANGUAGES CXX)


find_package(ompl REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
