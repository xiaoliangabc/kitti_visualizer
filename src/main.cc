#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kitti_visualizer");

  ros::spin();

  return 0;
}