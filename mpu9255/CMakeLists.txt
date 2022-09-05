cmake_minimum_required(VERSION 2.8.3)
project(mpu9255)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mpu9255_node src/MPU9255_node.cpp)
target_link_libraries(mpu9255_node ${catkin_LIBRARIES})

