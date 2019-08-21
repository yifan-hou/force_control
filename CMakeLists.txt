cmake_minimum_required(VERSION 2.8.3)
project(forcecontrol)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interfaces
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
  program_options
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp hardware_interfaces
  LIBRARIES forcecontrol_controller forcecontrol_hardware
)

include_directories(
  include
  /usr/include/eigen3
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

find_library(TIMER_LIB TimerLinux HINTS /usr/local/lib/RobotUtilities)
find_library(RUT Utilities HINTS /usr/local/lib/RobotUtilities)

###########
## Build ##
###########

add_library(forcecontrol_hardware src/forcecontrol_hardware.cpp)
target_link_libraries(forcecontrol_hardware ${RUT})

add_library(forcecontrol_controller src/forcecontrol_controller.cpp)
target_link_libraries(forcecontrol_controller ${RUT})

add_executable(forcecontrol_node src/main.cpp)
target_link_libraries(forcecontrol_node forcecontrol_hardware forcecontrol_controller
  ${TIMER_LIB} ${catkin_LIBRARIES}
  )