cmake_minimum_required(VERSION 3.0.2)
project(spaces)

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME}
    src/pandaControlTorque.cpp
)
target_link_libraries(${PROJECT_NAME} ompl)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/kdmp
)

