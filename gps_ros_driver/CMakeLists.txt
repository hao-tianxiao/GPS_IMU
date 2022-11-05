cmake_minimum_required(VERSION 2.8.3)
project(gps_ros_driver)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED 
			roscpp 
			sensor_msgs 
			nav_msgs 
			geometry_msgs
			gps_msgs
)

include_directories(
	  include
	  ${EIGEN3_INCLUDE_DIR}
	  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(gps_ros_driver src/gps_ros_driver.cpp)

target_link_libraries(gps_ros_driver ${catkin_LIBRARIES})
