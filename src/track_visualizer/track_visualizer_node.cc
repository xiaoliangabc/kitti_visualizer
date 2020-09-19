#include "track_visualizer/track_visualizer.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "track_visualizer_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  kitti_visualizer::TrackVisualizer track_visualizer(nh, pnh);

  // get frame number
  int frame_size = track_visualizer.GetFrameNumber();

  ros::Rate loop_rate(20);
  int frame = 0;
  // while (ros::ok()) {
  //   track_visualizer.Visualizer(frame);
  //   frame = (++frame) % frame_size;

  //   loop_rate.sleep();
  // }

  track_visualizer.Visualizer(frame);
  ros::spin();

  return 0;
}
