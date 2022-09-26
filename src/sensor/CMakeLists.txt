cmake_minimum_required(VERSION 3.0.2)
project(sensor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  DEPENDS Eigen3 
  INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS}
)

include_directories(
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor src/sensorRosNode.cpp)
target_link_libraries(sensor ${catkin_LIBRARIES})

