cmake_minimum_required(VERSION 2.8.3)
project(marker_detection)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	roscpp
	rospy
	std_msgs
	image_transport
	tf
	)

find_package(OpenCV REQUIRED)
include_directories(OpenCV_INCLUDE_DIRS)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES picture_joystick
	CATKIN_DEPENDS roscpp rospy std_msgs
	DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)

add_library(marker_detection
	src/${PROJECT_NAME}/marker_detection.cpp
	)
add_executable(marker_detection_node
	src/marker_detection_node.cpp
	)
add_executable(keyjoy
	src/keyjoy.cpp
	)

target_link_libraries(marker_detection_node
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	)
target_link_libraries(keyjoy
	${catkin_LIBRARIES}
	)
