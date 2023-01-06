cmake_minimum_required(VERSION 3.0.2)
project(models)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
)

add_library( ${PROJECT_NAME}
    src/CoriolisMatrix.cpp
    src/FrictionTorque.cpp
    src/GravityVector.cpp
    src/MassMatrix.cpp 
)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/kdmp
)

