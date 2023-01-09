cmake_minimum_required(VERSION 3.5)
project(kdmp VERSION 0.01 LANGUAGES CXX)

set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    "${CMAKE_CURRENT_SOURCE_DIR}/kdmp/CMakeModules")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

set(KDMP_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src;${CMAKE_CURRENT_BINARY_DIR}/src")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(jacobi)

include_directories(
    "${OMPL_INCLUDE_DIRS}"
    "${KDMP_INCLUDE_DIRS}")

add_subdirectory(src)
