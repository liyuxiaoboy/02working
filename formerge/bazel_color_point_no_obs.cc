/******************************************************************************
 * Copyright 2018 The MoonX Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "modules/localization/msf/common/io/velodyne_utility.h"

DEFINE_string(pcd_poses_path, "", "pcd poses path");
DEFINE_string(img_folder, "", "image folder");
DEFINE_string(lidar_poses_at_img_time, "", "lidar poses at image time");
DEFINE_string(lidar_cam_intrinsics, "", "lidar camera intrinsics");
DEFINE_string(lidar_cam_extrinsics, "", "lidar camera extrinsics");
DEFINE_string(output_pcd_folder, "", "output pcd folder");
DEFINE_string(range, "0", "points radius limitation, 0 means no limitation");

struct CamCalib {
  cv::Mat K;
  cv::Mat D;
};

inline int GetNum(const std::string &path) {
  int num = 0;
  int j = 1;
  for (int i = 5;; i++) {
    char tmp = path[path.length() - i];
    if (tmp < '0' || tmp > '9') break;
    num += (tmp - '0') * j;
    j = j * 10;
  }
  return num;
}

inline std::string GetDirectory(std::string file_path) {
  unsigned int dir_length = file_path.length();
  while (dir_length > 0 && file_path[dir_length - 1] != '/') {
    dir_length--;
  }
  return file_path.substr(0, dir_length);
}
//当前路径下搜索pcd
void GetPcdFilesAndPoses(const std::string &path,
                         std::map<int64, std::string> *files,
                         std::map<int64, Eigen::Affine3d> *poses) {
  std::vector<Eigen::Affine3d> poses_vec;
  std::vector<double> time_stamps;
  std::vector<unsigned int> pcd_indices;
  apollo::localization::msf::velodyne::LoadPcdPoses(path, &poses_vec,
                                                    &time_stamps, &pcd_indices);
  for (unsigned int i = 0; i < time_stamps.size(); i++) {
    int64 pcd_time = time_stamps[i] * 1e6;
    (*files)[pcd_time] =
        GetDirectory(path) + std::to_string(pcd_indices[i]) + ".pcd";
    (*poses)[pcd_time] = poses_vec[i];
  }
}
//image路径下png
void GetImgFilesAndPoses(const std::string &image_path,
                         std::map<int64, std::string> *files,
                         const std::string &poses_path,
                         std::map<int64, Eigen::Affine3d> *poses) {
  std::vector<Eigen::Affine3d> poses_vec;
  std::vector<double> time_stamps;
  std::vector<unsigned int> pcd_indices;
  apollo::localization::msf::velodyne::LoadPcdPoses(poses_path, &poses_vec,
                                                    &time_stamps, &pcd_indices);
  for (unsigned int i = 0; i < time_stamps.size(); i++) {
    int64 pcd_time = time_stamps[i] * 1e6;
    (*files)[pcd_time] = image_path + std::to_string(pcd_indices[i]) + ".png";
    (*poses)[pcd_time] = poses_vec[i];
  }
}

bool GetClosestImage(int64 pcd_time,
                     const std::map<int64, std::string> &img_map,
                     std::map<int64, std::string>::iterator img_iter,
                     int64 *img_time) {
  if (img_iter == img_map.end() || pcd_time < img_iter->first) return false;

  if (pcd_time == img_iter->first) {
    *img_time = img_iter->first;
    return true;
  }

  auto img_next = img_iter;
  img_next++;
  while (pcd_time > img_iter->first && img_next != img_map.end() &&
         pcd_time > img_next->first) {
    img_iter++;
    img_next++;
  }
  if (pcd_time > img_iter->first && pcd_time < img_next->first) {
    *img_time =
        (((pcd_time - img_iter->first) - (img_next->first - pcd_time)) > 0)
            ? img_next->first
            : img_iter->first;
    return true;
  }
  return false;
}

bool LoadExtrinsic(const std::string &file_path, Eigen::Affine3d *extrinsic) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["transform"]) {
    if (config["transform"]["translation"]) {
      extrinsic->translation()(0) =
          config["transform"]["translation"]["x"].as<double>();
      extrinsic->translation()(1) =
          config["transform"]["translation"]["y"].as<double>();
      extrinsic->translation()(2) =
          config["transform"]["translation"]["z"].as<double>();
      if (config["transform"]["rotation"]) {
        double qx = config["transform"]["rotation"]["x"].as<double>();
        double qy = config["transform"]["rotation"]["y"].as<double>();
        double qz = config["transform"]["rotation"]["z"].as<double>();
        double qw = config["transform"]["rotation"]["w"].as<double>();
        extrinsic->linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        return true;
      }
    }
  }
  return false;
}

bool LoadIntrinsic(const std::string &file_path, CamCalib *intrinsic) {
  YAML::Node config = YAML::LoadFile(file_path);
  if (config["K"] && config["D"]) {
    std::vector<double> K = config["K"].as<std::vector<double>>();
    std::vector<double> D = config["D"].as<std::vector<double>>();
    cv::Mat intrisicMat(3, 3, cv::DataType<double>::type);  // Intrisic matrix
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        intrisicMat.at<double>(i, j) = K[i * 3 + j];
      }
    }
    cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);
    for (int i = 0; i < 5; i++) {
      distCoeffs.at<double>(i) = D[i];
    }
    intrinsic->K = intrisicMat;
    intrinsic->D = distCoeffs;
    return true;
  }
  return false;
}

void ColorOnePointCloud(const std::string &pcd_file_name,
                        const std::string &image_file_name,
                        const CamCalib &intrinsic,
                        const Eigen::Affine3d &extrinsic, double range,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out) {
  cv::Mat image;
  image = cv::imread(image_file_name, CV_LOAD_IMAGE_COLOR);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_name, *cloud_src);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud_src, *cloud_cam, extrinsic);

  std::vector<cv::Point3d> objectPoints;
  for (size_t i = 0; i < cloud_cam->points.size(); i++) {
    objectPoints.push_back(cv::Point3d(cloud_cam->points[i].x,
                                       cloud_cam->points[i].y,
                                       cloud_cam->points[i].z));
  }
  std::vector<cv::Point2d> imagePoints;
  cv::Mat rVec(3, 1, cv::DataType<double>::type);
  rVec.at<double>(0) = 0;
  rVec.at<double>(1) = 0;
  rVec.at<double>(2) = 0;
  cv::Mat tVec(3, 1, cv::DataType<double>::type);
  tVec.at<double>(0) = 0;
  tVec.at<double>(1) = 0;
  tVec.at<double>(2) = 0;
  cv::projectPoints(objectPoints, rVec, tVec, intrinsic.K, intrinsic.D,
                    imagePoints);

  cv::Mat depth_2 = cv::Mat::zeros(image.rows, image.cols, CV_32FC1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cam_rgb(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  for (size_t i = 0; i < imagePoints.size(); i++) {

    int col = static_cast<int>(std::round(imagePoints[i].x));
    int row = static_cast<int>(std::round(imagePoints[i].y));
    auto x = cloud_cam->points[i].x;
    auto y = cloud_cam->points[i].y;
    auto z = cloud_cam->points[i].z;

    if (z <= 0 || col < 0 || col >= image.cols || row < 0 || row >= image.rows)
      continue;

    bool hide_flag = false;

    if( image.at<cv::Vec3b>(row, col)[2]==0 && image.at<cv::Vec3b>(row, col)[1]==0 && image.at<cv::Vec3b>(row, col)[0]==0)
               {
                   continue;
               }
               
    if (depth_2.at<float>(row, col) == 0)
      depth_2.at<float>(row, col) = x * x + y * y + z * z;
    if (x * x + y * y + z * z < depth_2.at<float>(row, col)) {
      depth_2.at<float>(row, col) = x * x + y * y + z * z;
      hide_flag = true;
    }

    if ((!range || (x * x + y * y + z * z < range * range)) && !hide_flag) {
      pcl::PointXYZRGB point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.r = image.at<cv::Vec3b>(row, col)[2];
      point.g = image.at<cv::Vec3b>(row, col)[1];
      point.b = image.at<cv::Vec3b>(row, col)[0];
      cloud_cam_rgb->points.push_back(point);
    }
  }
  pcl::transformPointCloud(*cloud_cam_rgb, *cloud_out, extrinsic.inverse());
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::string pcd_path = FLAGS_pcd_poses_path;
  std::string img_path = FLAGS_img_folder;
  std::string lidar_poses_at_img_time = FLAGS_lidar_poses_at_img_time;
  std::string output_pcd_path = FLAGS_output_pcd_folder;
  double range = std::stod(FLAGS_range);
  std::string extrinsic_path = FLAGS_lidar_cam_extrinsics;
  std::string intrinsic_path = FLAGS_lidar_cam_intrinsics;

  std::map<int64, std::string> pcd_map;
  std::map<int64, std::string> img_map;
  std::map<int64, Eigen::Affine3d> pcd_poses;
  std::map<int64, Eigen::Affine3d> img_time_poses;

  GetPcdFilesAndPoses(pcd_path, &pcd_map, &pcd_poses);
  GetImgFilesAndPoses(img_path, &img_map, lidar_poses_at_img_time,
                      &img_time_poses);
//外參
  Eigen::Affine3d extrinsic_l_c, extrinsic_c_l;
  if (!LoadExtrinsic(extrinsic_path, &extrinsic_c_l)) {
    std::cerr << "Load lidar extrinsic failed." << std::endl;
    exit(1);
  }
  extrinsic_l_c = extrinsic_c_l.inverse();
//内参
  CamCalib intrinsic;
  if (!LoadIntrinsic(intrinsic_path, &intrinsic)) {
    std::cerr << "Load lidar intrinsic failed." << std::endl;
    exit(1);
  }
//timestamps+path
  auto img_iter = img_map.begin();
  for (auto pcd_iter = pcd_map.begin(); pcd_iter != pcd_map.end(); pcd_iter++) {
    int64 img_time;
    int64 pcd_time = pcd_iter->first;
    std::string pcd_file = pcd_iter->second;
    std::ifstream fin(pcd_file);
    if (!fin) {
        continue;
    }
    if (!GetClosestImage(pcd_time, img_map, img_iter, &img_time)) continue;
    std::string image_file = img_map[img_time];

    Eigen::Affine3d affine_p_i;
    affine_p_i = pcd_poses[pcd_time].inverse() * img_time_poses[img_time];
    Eigen::Affine3d affine_l_c = extrinsic_l_c * affine_p_i;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_lid_rgb(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    ColorOnePointCloud(pcd_file, image_file, intrinsic, affine_l_c, range,
                       cloud_lid_rgb);

    if (!cloud_lid_rgb->points.empty()) {
      cloud_lid_rgb->width = 1;
      cloud_lid_rgb->height = cloud_lid_rgb->points.size();
      std::string name = std::to_string(GetNum(pcd_iter->second));
      pcl::io::savePCDFileBinary(output_pcd_path + name + ".pcd",
                                *cloud_lid_rgb);
      std::cout << "Saved " << cloud_lid_rgb->points.size()
                << " data points to " << name << ".pcd." << std::endl;
    }
  }
  return 0;
}
