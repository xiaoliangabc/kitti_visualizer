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

## Declare a C++ executable
add_executable(object_visualizer_node src/object_visualizer/object_visualizer.cc src/object_visualizer/object_visualizer_node.cc)
add_executable(track_visualizer_node src/track_visualizer/track_visualizer.cc src/track_visualizer/track_visualizer_node.cc)

## Specify libraries to link a library or executable target against
target_link_libraries(object_visualizer_node ${catkin_LIBRARIES})
target_link_libraries(object_visualizer_node ${OpenCV_LIBRARIES})
target_link_libraries(track_visualizer_node ${catkin_LIBRARIES})
target_link_libraries(track_visualizer_node ${OpenCV_LIBRARIES})

## This setting causes Qt's "MOC" generation to happen automatically
set(CMAKE_AUTOMOC ON)

## Use the Qt version that rviz used so they are compatible
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## Define QT_NO_KEYWORDS, avoid defining "emit", "slots", etc
add_definitions(-DQT_NO_KEYWORDS)

## Specify the list of source files
set(SRC_FILES
  src/common/rviz_command_button.cc
  ${QT_MOC}
)

## Specify the list of header files
set(HEADER_FILES
   include/common/rviz_command_button.h
)

## Declare library
add_library(${PROJECT_NAME}_button ${SRC_FILES} ${HEADER_FILES})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_button ${QT_LIBRARIES} ${catkin_LIBRARIES})
