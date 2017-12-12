cmake_minimum_required(VERSION 2.8.3)
project(forcecontrol)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  egm
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
  LIBRARIES forcecontrol_controller forcecontrol_hardware
)

include_directories(
  include
  /usr/include/eigen3
  ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ~/Git/ZZJHand/include/
)

include_directories(${egm_INCLUDE_DIRS})

###########
## Build ##
###########
# catkin_package(
#   INCLUDE_DIRS include ${PROJECT_BINARY_DIR}/
#   LIBRARIES forcecontrol_controller forcecontrol_hardware
#   CATKIN_DEPENDS roscpp ati_netft_hardware egm
#   )

add_library(ati_netft_hardware src/ati_netft_hardware.cpp)
target_link_libraries(ati_netft_hardware netft_rdt_driver ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(forcecontrol_hardware src/forcecontrol_hardware.cpp)
target_link_libraries(forcecontrol_hardware ati_netft_hardware egm)

add_library(forcecontrol_controller src/forcecontrol_controller.cpp)
# target_link_libraries(forcecontrol_controller ati_netft_hardware egm)

add_executable(regrasp_node src/regrasp.cpp)
target_link_libraries(regrasp_node forcecontrol_hardware forcecontrol_controller
  /usr0/home/yifanh/Git/ZZJHand/build/libHand.so
  # libEposCmd.so
  )


#############
## Install ##
#############
# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )



# Mark executables and/or libraries for installation
install(TARGETS forcecontrol_controller forcecontrol_hardware
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



