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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "gflags/gflags.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"

DEFINE_string(pcd_files_folder, "", "pcd files folder");
DEFINE_string(pcd_poses_path, "", "pcd poses path");
DEFINE_string(output_pcd_path, "", "output pcd path");

int GetNum(const std::string &path) {
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

void GetPose(const std::string &path, std::map<int, Eigen::Affine3d> *map) {
  std::vector<Eigen::Affine3d> poses_vec;
  std::vector<double> time_stamps;
  std::vector<unsigned int> pcd_indices;
  apollo::localization::msf::velodyne::LoadPcdPoses(path, &poses_vec,
                                                    &time_stamps, &pcd_indices);
  for (unsigned int i = 0; i < time_stamps.size(); i++) {
    (*map)[pcd_indices[i]] = poses_vec[i];
  }
}

// get tranform from index_1 to index_2
void GetTransform(int idx_1, int idx_2, Eigen::Matrix4d *T,
                  std::map<int, double[7]> *poses_map) {
  Eigen::Quaterniond q_1((*poses_map)[idx_1][6], (*poses_map)[idx_1][3],
                         (*poses_map)[idx_1][4], (*poses_map)[idx_1][5]);
  Eigen::Quaterniond q_2((*poses_map)[idx_2][6], (*poses_map)[idx_2][3],
                         (*poses_map)[idx_2][4], (*poses_map)[idx_2][5]);

  Eigen::Affine3d p_1, p_2;
  p_1.translation() = Eigen::Vector3d(
      (*poses_map)[idx_1][0], (*poses_map)[idx_1][1], (*poses_map)[idx_1][2]);
  p_1.linear() = q_1.matrix();
  p_2.translation() = Eigen::Vector3d(
      (*poses_map)[idx_2][0], (*poses_map)[idx_2][1], (*poses_map)[idx_2][2]);
  p_2.linear() = q_2.matrix();
  auto mat = p_2.inverse() * p_1;
  *T = mat.matrix();
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::string pcd_dir = FLAGS_pcd_files_folder;
  std::string odom_path = FLAGS_pcd_poses_path;
  std::string output_path = FLAGS_output_pcd_path;

  std::map<int, Eigen::Affine3d> poses_map;
  GetPose(odom_path, &poses_map);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  int base_index = 0;
  for (auto iter = poses_map.begin(); iter != poses_map.end(); iter++) {
    std::string path_tmp = pcd_dir + std::to_string(iter->first) + ".pcd";
    std::ifstream fin(path_tmp);
    if (!fin) {
      continue;
    }
    if (base_index == 0) {
      base_index = iter->first;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::io::loadPCDFile(path_tmp, *cloud_tmp);
      *cloud_sum = *cloud_tmp;
    } else {
      Eigen::Affine3d trans;
      trans = poses_map[base_index].inverse() * poses_map[iter->first];

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::io::loadPCDFile(path_tmp, *cloud_tmp);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp_out(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud(*cloud_tmp, *cloud_tmp_out, trans);

      *cloud_sum += *cloud_tmp_out;
    }
  }

  pcl::io::savePCDFileBinary(output_path, *cloud_sum);
  std::cout << "Saved " << cloud_sum->points.size() << " data points to "
            << output_path << std::endl;

  return (0);
}
