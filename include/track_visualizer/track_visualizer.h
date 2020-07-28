#ifndef KITTI_VISUALIZER_TRACK_VISUALIZER_TRACK_VISUALIZER_H_
#define KITTI_VISUALIZER_TRACK_VISUALIZER_TRACK_VISUALIZER_H_

#include <ros/ros.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/String.h>

#include <tf/transform_datatypes.h>

#include "common/transform_utils.h"
#include "common/utils.h"

namespace kitti_visualizer {

class TrackVisualizer {
 public:
  TrackVisualizer(ros::NodeHandle nh, ros::NodeHandle pnh);

  // Visualize object data
  void Visualizer(const int& frame);

  // Get frame number
  int GetFrameNumber();

 private:
  // Visualize point cloud
  void PointCloudVisualizer(const std::string& file_prefix,
                            const ros::Publisher publisher);

  // Visualize image
  void ImageVisualizer(const std::string& file_prefix,
                       const ros::Publisher publisher);

  // Draw 2D bounding boxes in image
  void Draw2DBoundingBoxes(const std::string& file_prefix, cv::Mat& raw_image);

  // Visualize 3D bounding boxes
  void BoundingBoxesVisualizer(const std::string& file_prefix,
                               const ros::Publisher publisher);

  // Transform 3D bounding boxes form camera to velodyne
  jsk_recognition_msgs::BoundingBoxArray TransformBoundingBoxes(
      const std::vector<std::vector<float>> detections,
      const std::string& file_prefix);

  // Parse detections from file
  std::vector<std::vector<float>> ParseTracks(const std::string& file_prefix);

  // Subscribe command from Rviz
  void CommandButtonCallback(const std_msgs::String::ConstPtr& in_command);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscriber
  ros::Subscriber sub_command_button_;

  // Publisher
  ros::Publisher pub_point_cloud_;
  ros::Publisher pub_image_;
  ros::Publisher pub_bounding_boxes_;

  // Object data path
  std::string data_path_;
  std::string dataset_;

  // Scene
  std::string scene_;

  // Frame
  int frame_size_;
  int current_frame_;
};

}  // namespace kitti_visualizer

#endif  // KITTI_VISUALIZER_TRACK_VISUALIZER_TRACK_VISUALIZER_H_
