cmake_minimum_required(VERSION 3.5)
project(force_control)

add_definitions(-std=c++17)

find_package(Eigen3 3.4 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
  include
)

find_library(YAMLLib yaml-cpp HINTS /usr/local/lib)
find_library(RUT Utilities HINTS ${CMAKE_INSTALL_PREFIX}/lib/RobotUtilities)

add_library(FORCE_CONTROLLERS SHARED src/admittance_controller.cpp src/config_deserialize.cpp)
  target_link_libraries(FORCE_CONTROLLERS ${RUT} ${YAMLLib})

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)

install(TARGETS FORCE_CONTROLLERS
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)