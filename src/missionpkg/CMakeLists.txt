cmake_minimum_required(VERSION 2.8.3)
project(missionpkg)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS 
	cv_bridge 
	image_transport 
	message_runtime 
	sensor_msgs
)

find_package(OpenCV)

set (CMAKE_CXX_STANDARD 11)

include_directories(
	include 
	${catkin_INCLUDE_DIRS} 
	${OpenCV_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/el_det_fi.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(offboard1 src/offb1.cpp)
target_link_libraries(offboard1 ${catkin_LIBRARIES})
add_executable(offboard2 src/offb2.cpp)
target_link_libraries(offboard2 ${catkin_LIBRARIES})