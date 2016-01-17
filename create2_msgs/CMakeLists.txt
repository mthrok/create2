cmake_minimum_required(VERSION 2.8.3)
project(create2_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  Wall.msg
  Song.msg
  Cliff.msg
  Button.msg
  Travel.msg
  Battery.msg
  Request.msg
  IR_Opcodes.msg
  LightBumper.msg
  MotorCurrent.msg
  EncoderCounts.msg
  ChargingSource.msg
  BumpsWheeldrops.msg
  WheelOvercurrents.msg
  Status.msg
  Control.msg
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
