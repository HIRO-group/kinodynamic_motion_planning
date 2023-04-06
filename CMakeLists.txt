cmake_minimum_required(VERSION 3.0.2)
project(kdmp_ros)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 11)
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
    geometry_msgs
    message_generation
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    ompl
)
find_package(Eigen3 REQUIRED)



set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules"
    "${CMAKE_CURRENT_SOURCE_DIR}/kdmp/CMakeModules")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

set(KDMP_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src;${CMAKE_CURRENT_BINARY_DIR}/src")

add_message_files(
  FILES
  PandaControlCmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
    include
    ${CMAKE_INSTALL_INCLUDEDIR}
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/panda_edge_data_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(src)

add_library( robot_interfaces
    robot_interfaces/pandaMoveitInterface.cpp
)
target_link_libraries(robot_interfaces kdmp ${catkin_LIBRARIES})
target_include_directories(robot_interfaces PUBLIC 
                           "src/kdmp/include"
                          )

add_library(environments 
    env/moveItEnv.cpp)  
target_link_libraries(environments ${catkin_LIBRARIES} kdmp)
      
catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(omplRRT execs/omplRRT.cpp)
target_link_libraries(omplRRT ${catkin_LIBRARIES} robot_interfaces kdmp environments)
target_include_directories(omplRRT PUBLIC
                           "include/kdmp_ros/")

add_executable(test_propogate_fn test/test_propogate.cpp)
target_link_libraries(test_propogate_fn ${catkin_LIBRARIES} robot_interfaces kdmp environments)
target_include_directories(test_propogate_fn PUBLIC
                            "include/kdmp_ros/")