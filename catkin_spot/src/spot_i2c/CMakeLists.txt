cmake_minimum_required(VERSION 3.0.2)
project(spot_i2c)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  )

add_message_files(
  DIRECTORY msg
  FILES imu.msg tof.msg
  )

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJJECT_NAME}
#CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
CATKIN_DEPENDS message_runtime
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${i2cpwm_board_INCLUDE_DIRS}
  ${spot_i2c_INCLUDE_DIRS}
  )


link_directories(${catkin_LIBRARY_DIRS})






add_executable( spot_i2c_node
  src/lcd.cpp
  src/adc.cpp
  src/mpu9250.cpp
  src/tof.cpp
  src/main.cpp
  )

target_link_libraries(spot_i2c_node
  i2c
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  wiringPi
  )


add_dependencies(spot_i2c_node spot_i2c_generate_messages_cpp)






