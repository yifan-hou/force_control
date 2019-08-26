add_executable(forcecontrol_node main.cpp)
target_link_libraries(forcecontrol_node forcecontrol_hardware forcecontrol_controller
  ${TIMER_LIB} ${catkin_LIBRARIES}
)
