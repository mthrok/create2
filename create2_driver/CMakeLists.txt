cmake_minimum_required(VERSION 2.8.3)
project(create2_driver)

find_package(catkin REQUIRED COMPONENTS
  create2_msgs
  roslaunch
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS    include
  LIBRARIES       create2
  CATKIN_DEPENDS  create2_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(create2
  src/serial.cxx
  src/create2.cxx
)

add_executable(create2_driver
  src/create2_driver.cxx
)

add_dependencies(create2_driver
  create2_msgs_generate_messages_cpp
)

target_link_libraries(create2_driver
  create2
  ${catkin_LIBRARIES}
)
