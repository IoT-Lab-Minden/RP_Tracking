cmake_minimum_required(VERSION 3.0.2)
project(rp_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
  eigen_conversions
  jsk_recognition_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rp_tracking
#  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  "${CMAKE_CURRENT_SOURCE_DIR}/include/rp_tracking"
  ${catkin_INCLUDE_DIRS}
)

add_executable(background_removal src/background_removal.cpp)
target_link_libraries(background_removal ${catkin_LIBRARIES})

add_executable(cluster_detection src/cluster_detection.cpp)
target_link_libraries(cluster_detection ${catkin_LIBRARIES})

add_executable(ground_removal src/ground_removal.cpp)
target_link_libraries(ground_removal ${catkin_LIBRARIES})

add_executable(robot_detection src/robot_detection.cpp)
target_link_libraries(robot_detection ${catkin_LIBRARIES})
