cmake_minimum_required (VERSION 2.6)
project (Egm_controller)


include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

# # The version number.
# set (Tutorial_VERSION_MAJOR 1)
# set (Tutorial_VERSION_MINOR 0)
 
# # configure a header file to pass some of the CMake settings
# # to the source code
# configure_file (
#   "${PROJECT_SOURCE_DIR}/TutorialConfig.h.in"
#   "${PROJECT_BINARY_DIR}/TutorialConfig.h"
#   )
 
# # add the binary tree to the search path for include files
# # so that we will find TutorialConfig.h
# include_directories("${PROJECT_BINARY_DIR}")

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/include")

# add libraries
add_library(PracticalSocket "include/PracticalSocket/PracticalSocket.cpp")
add_library(Egm_proto "src/egm.pb.cc")
# add_subdirectory(include/PracticalSocket)
# add_subdirectory(src/PracticalSocket)

# add the executable
add_executable(egm_server_node src/egm_server_node.cpp)
target_link_libraries(egm_server_node PracticalSocket Egm_proto
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${PROTOBUF_LIBRARY}
)
