cmake_minimum_required(VERSION 2.8.3)
project(opencv-for-catkin-sample)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  compressed_depth_image_transport
  compressed_image_transport
  cv_bridge
  image_transport
  kinect2_bridge
  #libpcl-dev
  message_filters
  #aruco
  #aruco_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES buoy
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(opencv-test src/test.cpp) # src/aruco.cpp)

#add_executable(aruco-test src/aruco.cpp)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
find_package(OpenCV)
find_package(PCL)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(opencv-test 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${kinect2_bridge_LIBRARIES}
)

# target_link_libraries(beer_locator
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   # ${PCL_LIBRARIES}
#   ${kinect2_bridge_LIBRARIES}
# )

