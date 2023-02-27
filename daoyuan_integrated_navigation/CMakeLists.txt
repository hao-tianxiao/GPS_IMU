cmake_minimum_required(VERSION 3.0.2)
project(daoyuan_integrated_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gps_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  serial
  tf
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3	
  /usr/include/yaml-cpp
)

add_executable(daoyuan_integrated_navigation main.cpp src/daoyuan_integrated_navigation.cpp)
target_link_libraries(daoyuan_integrated_navigation ${catkin_LIBRARIES})
add_dependencies(daoyuan_integrated_navigation ${catkin_EXPORTED_TARGETS} )
