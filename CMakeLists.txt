cmake_minimum_required(VERSION 2.8.3)
project(kitti_visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  rviz
  sensor_msgs
  visualization_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport pcl_conversions pcl_ros roscpp rospy rviz sensor_msgs visualization_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
