#ifndef KITTI_VISUALIZER_COMMON_UTILS_H_
#define KITTI_VISUALIZER_COMMON_UTILS_H_

#include <ros/ros.h>

#include <dirent.h>
#include <fstream>
#include <iostream>

#include <pcl_ros/point_cloud.h>

namespace kitti_visualizer {
static int FolderFilesNumber(std::string path) {
  DIR *dir;
  struct dirent *ent;
  int files_number = 0;
  if ((dir = opendir(path.c_str())) != NULL) {
    // Print all the files and directories within directory
    while ((ent = readdir(dir)) != NULL) {
      if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
        continue;
      files_number++;
    }
    closedir(dir);
  } else {
    // Could not open directory
    ROS_ERROR("Could not open directory: %s", path.c_str());
    ros::shutdown();
  }

  return files_number;
}

static void ReadPointCloud(const std::string &in_file,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud) {
  // Load point cloud from .bin file
  std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
  if (!input.good()) {
    ROS_ERROR("Could not read file: %s", in_file.c_str());
    exit(EXIT_FAILURE);
  }
  input.seekg(0, std::ios::beg);

  // Transform .bin file to pcl cloud
  for (size_t i = 0; input.good() && !input.eof(); i++) {
    pcl::PointXYZI pt;
    // Read data
    input.read((char *)&pt.x, 3 * sizeof(float));
    input.read((char *)&pt.intensity, sizeof(float));
    raw_cloud->points.push_back(pt);
  }
  input.close();
}

// static void ReadCalibMatrix(const std::string &file_path,
//                             const std::string &matrix_name,
//                             Eigen::MatrixXd &trans_matrix) {
//   // Open calib file
//   std::ifstream ifs(file_path);
//   if (!ifs) {
//     ROS_ERROR("File %s does not exist", file_path.c_str());
//     ros::shutdown();
//   }

//   // Read matrix
//   std::string temp_str;
//   while (std::getline(ifs, temp_str)) {
//     std::istringstream iss(temp_str);
//     std::string name;
//     iss >> name;
//     if (name == matrix_name) {
//       float temp_float;
//       for (int i = 0; i < trans_matrix.rows(); ++i) {
//         for (int j = 0; j < trans_matrix.cols(); ++j) {
//           if (iss.rdbuf()->in_avail() != 0) {
//             iss >> temp_float;
//             trans_matrix(i, j) = temp_float;
//           }
//         }
//       }
//     }
//   }
// }

static void ReadCalibMatrix(const std::string &file_path,
                            const std::string &matrix_name,
                            Eigen::MatrixXd &trans_matrix) {
  // Open calib file
  std::ifstream ifs(file_path);
  if (!ifs) {
    ROS_ERROR("File %s does not exist", file_path.c_str());
    ros::shutdown();
  }

  // Read matrix
  std::string temp_str;
  while (std::getline(ifs, temp_str)) {
    std::istringstream iss(temp_str);
    std::string name;
    iss >> name;
    if (name == matrix_name) {
      if (matrix_name == "P2:") {
        trans_matrix = Eigen::MatrixXd::Zero(3, 4);
        float temp_float;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 4; ++j) {
            iss >> temp_float;
            trans_matrix(i, j) = temp_float;
          }
        }
        return;
      } else if (matrix_name == "R0_rect:") {
        trans_matrix = Eigen::MatrixXd::Zero(4, 4);
        float temp_float;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            iss >> temp_float;
            trans_matrix(i, j) = temp_float;
          }
        }
        trans_matrix(3, 3) = 1.0;
        return;
      } else if (matrix_name == "Tr_velo_to_cam:") {
        trans_matrix = Eigen::MatrixXd::Zero(4, 4);
        float temp_float;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 4; ++j) {
            iss >> temp_float;
            trans_matrix(i, j) = temp_float;
          }
        }
        trans_matrix(3, 3) = 1.0;
        return;
      } else if (matrix_name == "Tr_cam_to_road:") {
        trans_matrix = Eigen::MatrixXd::Zero(4, 4);
        float temp_float;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 4; ++j) {
            iss >> temp_float;
            trans_matrix(i, j) = temp_float;
          }
        }
        trans_matrix(3, 3) = 1.0;
        return;
      }
    }
  }
  ROS_ERROR("Transform matrix %s does not exist", matrix_name.c_str());
  ros::shutdown();
}

}  // namespace kitti_visualizer

#endif  // KITTI_VISUALIZER_COMMON_UTILS_H_
