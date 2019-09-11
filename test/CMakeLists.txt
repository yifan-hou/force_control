add_executable(force_control_node main.cpp)
target_link_libraries(force_control_node force_control_hardware force_control_controller
  ${TIMER_LIB} ${catkin_LIBRARIES}
)
