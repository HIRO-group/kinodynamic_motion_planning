cmake_minimum_required(VERSION 3.0.2)
project(planners)

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
)

add_library( ${PROJECT_NAME}
  src/rrtConnectTp.cpp
)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/kdmp
)

