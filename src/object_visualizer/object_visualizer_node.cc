#include "object_visualizer/object_visualizer.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "object_visualizer_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  kitti_visualizer::ObjectVisualizer object_visualizer(nh, pnh);

  ros::Duration(3).sleep();
  object_visualizer.Visualizer();

  ros::spin();

  return 0;
}
