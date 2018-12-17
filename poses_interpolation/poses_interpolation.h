/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file poses_interpolation.h
 * @brief
 */

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_POSES_INTERPOLATION
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_POSES_INTERPOLATION

#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

/**
 * @namespace apollo::localization
 * @brief apollo::localization
 */
namespace apollo {
namespace localization {
namespace msf {

class PosesInterpolation {
 public:
  PosesInterpolation();
  void DoInterpolation(const std::string &input_poses_path,
                       const std::string &ref_timestamps_path,
                       const std::string &out_poses_path,
                       const std::string &extrinsic_path,
                       const std::string &gnss_best_pose_path,
                       const std::string &gnss_status_path,
                       const std::string &chassis_path);

 private:
  void LoadPCDTimestamp();
  void WritePCDPoses();
  std::vector<double> InterpolationByTime(
      const std::vector<std::pair<double, double>> &in_data);

  void PoseInterpolationByTime();
  void StdDevInterpolationByTime();
  void SatNumInterpolationByTime();
  void ChassisInterpolationByTime();

 private:
  std::string input_poses_path_;
  std::string ref_timestamps_path_;
  std::string out_poses_path_;
  std::string gnss_best_pose_path_;
  std::string gnss_status_path_;
  std::string chassis_path_;

  std::vector<unsigned int> indexes_;
  std::vector<double> timestamps_;
  std::vector<Eigen::Affine3d> poses_;
  std::vector<Eigen::Vector3d> std_dev_;
  std::vector<double> num_sats_;
  std::vector<Eigen::Affine3d> vdm_poses_;
  bool vdm_available_;

  Eigen::Affine3d velodyne_extrinsic_;
  std::vector<bool> done_;
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo

#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_POSES_INTERPOLATION
