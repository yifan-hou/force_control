cmake_minimum_required(VERSION 2.8.12)
project(egm_comm)
set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  matvec
)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR}	)
 

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/include/egm_comm")




###########
## Build ##
###########
# add_library(PracticalSocket "src/PracticalSocket.cpp")
# add_library(Egm_proto "src/egm.pb.cc")
# add_library(Timer "src/TimerLinux.cpp")
add_library(egm_comm 
	"src/EGMClass.cpp" 
	"src/PracticalSocket.cpp" 
	"src/egm.pb.cc" 
	"src/TimerLinux.cpp")

target_link_libraries(egm_comm	
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${PROTOBUF_LIBRARY}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES egm_comm
  CATKIN_DEPENDS roscpp matvec
  )

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

