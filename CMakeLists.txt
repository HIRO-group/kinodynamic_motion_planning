cmake_minimum_required(VERSION 3.0.2)
project(kdmp_ros)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

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

include_directories(
    include
    ${CMAKE_INSTALL_INCLUDEDIR}
    "${OMPL_INCLUDE_DIRS}"
    "${KDMP_INCLUDE_DIRS}"
    ${catkin_INCLUDE_DIRS}
)


add_subdirectory(src)

add_library( robot_interfaces
    robot_interfaces/pandaMoveitInterface.cpp
)

catkin_package()

add_executable(omplRRT execs/omplRRT.cpp)
target_link_libraries(omplRRT ${catkin_LIBRARIES} robot_interfaces ${OMPL_LIBRARIES} )