cmake_minimum_required(VERSION 2.8.3)
project(forcecontrol)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  egm_hardware
  netft_rdt_driver
  roscpp
  diagnostic_updater 
  diagnostic_msgs
  geometry_msgs
  message_generation
  controller_manager
  hardware_interface
  dynamic_reconfigure
)


find_package(Boost REQUIRED COMPONENTS 
  system 
  thread 
  program_options
)

add_service_files(
  FILES
  ResetThresholdLatch.srv
  SetSoftwareBias.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp diagnostic_updater diagnostic_msgs geometry_msgs message_runtime controller_manager hardware_interface dynamic_reconfigure
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

include_directories(${egm_hardware_INCLUDE_DIRS})

###########
## Build ##
###########

add_library(ati_netft_hardware src/ati_netft_hardware.cpp)
target_link_libraries(ati_netft_hardware netft_rdt_driver ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(forcecontrol_hardware src/forcecontrol_hardware.cpp)
target_link_libraries(forcecontrol_hardware ati_netft_hardware egm_hardware)

add_library(forcecontrol_controller src/forcecontrol_controller.cpp)
# target_link_libraries(forcecontrol_controller ati_netft_hardware egm_hardware)

add_executable(forcecontrol_node src/main.cpp)
target_link_libraries(forcecontrol_node forcecontrol_hardware forcecontrol_controller
	"/usr/local/lib/yifanlibrary/libTimerLinux.so")
