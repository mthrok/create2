cmake_minimum_required(VERSION 2.8.3)
project(create2_joystick_control)

find_package(catkin REQUIRED COMPONENTS
  create2_msgs
  joy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(create2_joystick_control
  src/create2_joystick_control.cpp
)

add_dependencies(create2_joystick_control
  create2_msgs_generate_messages_cpp
  joy_generate_messages_cpp
)

target_link_libraries(create2_joystick_control
  ${catkin_LIBRARIES}
)
