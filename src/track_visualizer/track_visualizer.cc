#include "track_visualizer/track_visualizer.h"

namespace kitti_visualizer {

TrackVisualizer::TrackVisualizer(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh) {
  pnh_.param<std::string>("data_path", data_path_, "");
  pnh_.param<std::string>("dataset", dataset_, "");
  pnh_.param<std::string>("scene", scene_, "");
  pnh_.param<int>("current_frame", current_frame_, 0);

  // Subscriber
  sub_command_button_ =
      nh_.subscribe("/kitti_visualizer/command_button", 2,
                    &TrackVisualizer::CommandButtonCallback, this);

  // Publisher
  pub_point_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "kitti_visualizer/object/point_cloud", 2);
  pub_image_ =
      nh_.advertise<sensor_msgs::Image>("kitti_visualizer/object/image", 2);
  pub_bounding_boxes_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(
      "kitti_visualizer/object/bounding_boxes", 2);
}

void TrackVisualizer::Visualizer(const int& frame) {
  // Get current file name
  std::ostringstream file_prefix;
  file_prefix << std::setfill('0') << std::setw(6) << frame;
  ROS_INFO("Visualizing frame %s ...", file_prefix.str().c_str());

  // Visualize point cloud
  PointCloudVisualizer(file_prefix.str(), pub_point_cloud_);

  // Visualize image
  ImageVisualizer(file_prefix.str(), pub_image_);

  // Visualize 3D bounding boxes
  BoundingBoxesVisualizer(file_prefix.str(), pub_bounding_boxes_);
}

void TrackVisualizer::PointCloudVisualizer(const std::string& file_prefix,
                                           const ros::Publisher publisher) {
  // Read point cloud
  std::string cloud_file_name = data_path_ + dataset_ + "/velodyne/" + scene_ +
                                "/" + file_prefix + ".bin";
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  ReadPointCloud(cloud_file_name, raw_cloud);

  // Publish point cloud
  raw_cloud->header.frame_id = "base_link";
  publisher.publish(raw_cloud);
}

void TrackVisualizer::ImageVisualizer(const std::string& file_prefix,
                                      const ros::Publisher publisher) {
  // Read image
  std::string image_file_name = data_path_ + dataset_ + "/image_02/" + scene_ +
                                "/" + file_prefix + ".png";
  cv::Mat raw_image = cv::imread(image_file_name.c_str());

  // Draw 2D bounding boxes in image
  Draw2DBoundingBoxes(file_prefix, raw_image);

  // Publish image
  sensor_msgs::ImagePtr raw_image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_image).toImageMsg();
  raw_image_msg->header.frame_id = "base_link";
  publisher.publish(raw_image_msg);
}

void TrackVisualizer::Draw2DBoundingBoxes(const std::string& file_prefix,
                                          cv::Mat& raw_image) {
  // Read bounding boxes data
  std::vector<std::vector<float>> tracks = ParseTracks(file_prefix);

  // Draw bounding boxes in image
  for (const auto track : tracks) {
    cv::rectangle(raw_image, cv::Point(track[4], track[5]),
                  cv::Point(track[6], track[7]), cv::Scalar(0, 255, 0), 2, 8,
                  0);
  }
}

void TrackVisualizer::BoundingBoxesVisualizer(const std::string& file_prefix,
                                              const ros::Publisher publisher) {
  // Read bounding boxes data
  std::vector<std::vector<float>> tracks = ParseTracks(file_prefix);

  // Transform bounding boxes to jsk_recognition_msgs
  jsk_recognition_msgs::BoundingBoxArray bounding_box_array =
      TransformBoundingBoxes(tracks, file_prefix);

  // Publish bounding boxes
  bounding_box_array.header.frame_id = "base_link";
  pub_bounding_boxes_.publish(bounding_box_array);
}

jsk_recognition_msgs::BoundingBoxArray TrackVisualizer::TransformBoundingBoxes(
    const std::vector<std::vector<float>> tracks,
    const std::string& file_prefix) {
  // Read transform matrixs from calib file
  std::string calib_file_name =
      data_path_ + dataset_ + "/calib/" + scene_ + ".txt";
  Eigen::MatrixXd trans_velo_to_cam = Eigen::MatrixXd::Identity(4, 4);
  ReadCalibMatrix(calib_file_name, "Tr_velo_cam", trans_velo_to_cam);
  Eigen::MatrixXd trans_cam_to_rect = Eigen::MatrixXd::Identity(4, 4);
  ReadCalibMatrix(calib_file_name, "R_rect", trans_cam_to_rect);

  // Set bounding boxes to jsk_recognition_msgs::BoundingBoxArray
  jsk_recognition_msgs::BoundingBoxArray bounding_box_array;
  for (const auto track : tracks) {
    jsk_recognition_msgs::BoundingBox bounding_box;
    // Bounding box id
    bounding_box.label = static_cast<int>(track[0]);
    // Bounding box position
    Eigen::Vector4d rect_position(track[11], track[12], track[13], 1.0);
    Eigen::MatrixXd velo_position = trans_velo_to_cam.inverse() *
                                    trans_cam_to_rect.inverse() * rect_position;
    bounding_box.pose.position.x = velo_position(0);
    bounding_box.pose.position.y = velo_position(1);
    bounding_box.pose.position.z = velo_position(2) + track[8] / 2.0;
    // Bounding box orientation
    tf::Quaternion bounding_box_quat =
        tf::createQuaternionFromRPY(0.0, 0.0, 0.0 - track[14]);
    tf::quaternionTFToMsg(bounding_box_quat, bounding_box.pose.orientation);
    // Bounding box dimensions
    bounding_box.dimensions.x = track[9];
    bounding_box.dimensions.y = track[10];
    bounding_box.dimensions.z = track[8];
    // Bounding box header
    bounding_box.header.stamp = ros::Time::now();
    bounding_box.header.frame_id = "base_link";
    bounding_box_array.boxes.push_back(bounding_box);
  }

  return bounding_box_array;
}

std::vector<std::vector<float>> TrackVisualizer::ParseTracks(
    const std::string& file_prefix) {
  // Open bounding boxes file
  std::string tracks_file_name;
  if (dataset_ == "training") {
    tracks_file_name = data_path_ + dataset_ + "/label_02/" + scene_ + ".txt";
  } else if (dataset_ == "testing") {
    tracks_file_name =
        data_path_ + dataset_ + "/results/" + file_prefix + ".txt";
  }
  std::ifstream tracks_file(tracks_file_name);
  if (!tracks_file) {
    ROS_ERROR("File %s does not exist", tracks_file_name.c_str());
    ros::shutdown();
  }

  // Parse tracks data
  std::vector<std::vector<float>> tracks;
  std::string line_str;
  while (getline(tracks_file, line_str)) {
    // Store std::string into std::stringstream
    std::stringstream line_ss(line_str);
    // Parse frame number
    std::string frame;
    getline(line_ss, frame, ' ');
    if (boost::lexical_cast<int>(frame) !=
        boost::lexical_cast<int>(file_prefix))
      continue;
    // Parse object data
    std::vector<float> track;
    // Parse object id
    std::string object_id;
    getline(line_ss, object_id, ' ');
    if (boost::lexical_cast<int>(object_id) < 0) continue;
    track.push_back(boost::lexical_cast<float>(object_id));
    // Parse object type
    std::string object_type;
    getline(line_ss, object_type, ' ');
    if (object_type == "DontCare") continue;
    std::string str;
    while (getline(line_ss, str, ' ')) {
      track.push_back(boost::lexical_cast<float>(str));
    }
    tracks.push_back(track);
  }

  return tracks;
}

void TrackVisualizer::CommandButtonCallback(
    const std_msgs::String::ConstPtr& in_command) {
  // Parse frame number form command
  if (in_command->data == "Next") {
    current_frame_ = (frame_size_ + current_frame_ + 1) % frame_size_;
  } else if (in_command->data == "Prev") {
    current_frame_ = (frame_size_ + current_frame_ - 1) % frame_size_;
  } else {
    int frame = std::stoi(in_command->data);
    if (frame >= 0 && frame < frame_size_)
      current_frame_ = frame;
    else
      ROS_ERROR("No frame %s", in_command->data.c_str());
  }

  // Visualize object data
  Visualizer(current_frame_);
}

int TrackVisualizer::GetFrameNumber() {
  // Get velodyne frame number
  int velo_frame_num =
      FolderFilesNumber(data_path_ + dataset_ + "/velodyne/" + scene_ + "/");

  // Get image_2 frame number
  int image_frame_num =
      FolderFilesNumber(data_path_ + dataset_ + "/image_02/" + scene_ + "/");

  // Assert velodyne and image
  ROS_ASSERT(velo_frame_num == image_frame_num);

  // Assign
  frame_size_ = velo_frame_num;

  return velo_frame_num;
}
}
