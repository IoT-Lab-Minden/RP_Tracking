cmake_minimum_required(VERSION 3.0.2)
project(rp_preprocessing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rp_preprocessing
#  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  "${CMAKE_CURRENT_SOURCE_DIR}/include/rp_preprocessing"
  ${catkin_INCLUDE_DIRS}
)

add_executable(consecutive_cloud_merge src/consecutive_cloud_merge.cpp)
target_link_libraries(consecutive_cloud_merge ${catkin_LIBRARIES})

add_executable(multi_cloud_merge src/multi_cloud_merge.cpp)
target_link_libraries(multi_cloud_merge ${catkin_LIBRARIES})
