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

#include "modules/localization/msf/local_tool/map_creation/poses_interpolation/poses_interpolation.h"
#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>
#include "modules/localization/msf/common/io/velodyne_utility.h"
#include "modules/localization/msf/local_tool/map_creation/poses_interpolation/vehicle_dynamic_model.h"

namespace apollo {
namespace localization {
namespace msf {
PosesInterpolation::PosesInterpolation() {}

void PosesInterpolation::DoInterpolation(const std::string &input_poses_path,
                                         const std::string &ref_timestamps_path,
                                         const std::string &out_poses_path,
                                         const std::string &extrinsic_path,
                                         const std::string &gnss_best_pose_path,
                                         const std::string &gnss_status_path,
                                         const std::string &chassis_path) {
  this->input_poses_path_ = input_poses_path;
  this->ref_timestamps_path_ = ref_timestamps_path;
  this->out_poses_path_ = out_poses_path;
  this->gnss_best_pose_path_ = gnss_best_pose_path;
  this->gnss_status_path_ = gnss_status_path;
  this->chassis_path_ = chassis_path;
  this->vdm_available_ = false;
  if (!velodyne::LoadExtrinsic(extrinsic_path, &velodyne_extrinsic_)) {
    std::cerr << "Load lidar extrinsic failed." << std::endl;
    exit(1);
  }
  // Load pcd timestamp
  LoadPCDTimestamp();
  // do interpolation
  PoseInterpolationByTime();
  StdDevInterpolationByTime();
  SatNumInterpolationByTime();
  ChassisInterpolationByTime();

  // Write pcd poses
  WritePCDPoses();
}

void PosesInterpolation::LoadPCDTimestamp() {
  FILE *file = fopen(ref_timestamps_path_.c_str(), "r");
  if (file) {
    unsigned int index;
    double timestamp;
    constexpr int kSize = 2;
    while (fscanf(file, "%u %lf\n", &index, &timestamp) == kSize) {
      timestamps_.push_back(timestamp);
      indexes_.push_back(index);
    }
    fclose(file);
  } else {
    std::cerr << "Can't open file to read: " << ref_timestamps_path_
              << std::endl;
    exit(1);
  }
  poses_.resize(indexes_.size());
  std_dev_.resize(indexes_.size());
  num_sats_.resize(indexes_.size());
  vdm_poses_.resize(indexes_.size());
  done_ = std::vector<bool>(indexes_.size(), true);
}

void PosesInterpolation::WritePCDPoses() {
  std::vector<velodyne::PcdPose> frames;
  auto prefix = boost::filesystem::canonical(input_poses_path_).parent_path();
  for (size_t i = 0; i < indexes_.size(); i++) {
    if (done_[i]) {
      if (!frames.empty()) {
        auto cur_pose = poses_[i];
        auto pre_pose = frames.back().pose;
        if ((cur_pose.translation() - pre_pose.translation()).norm() < 0.1) {
          continue;
        }
      }
      auto pcd_path = (prefix / (std::to_string(indexes_[i]) + ".pcd"));
      frames.push_back(velodyne::PcdPose(pcd_path.string(), timestamps_[i],
                                         poses_[i], std_dev_[i], num_sats_[i],
                                         false, Eigen::Affine3d::Identity(),
                                         false, vdm_available_, vdm_poses_[i]));
    }
  }
  velodyne::SaveSerialzedPoseList(frames, out_poses_path_);
}

std::vector<double> PosesInterpolation::InterpolationByTime(
    const std::vector<std::pair<double, double>> &in_data) {
  std::vector<double> out_data(timestamps_.size());
  std::vector<bool> done(timestamps_.size());
  for (size_t i = 0, head = 0; i < timestamps_.size(); i++) {
    while (head < in_data.size() && in_data[head].first < timestamps_[i]) {
      head++;
    }
    if (head > 0 && head < in_data.size()) {
      double pre_timestamp = in_data[head - 1].first;
      double cur_timestamp = in_data[head].first;
      double ref_timestamp = timestamps_[i];
      assert(ref_timestamp >= pre_timestamp && ref_timestamp <= cur_timestamp);
      assert(cur_timestamp > pre_timestamp);
      double t =
          (cur_timestamp - ref_timestamp) / (cur_timestamp - pre_timestamp);
      out_data[i] =
          in_data[head - 1].second * t + in_data[head].second * (1 - t);
      done[i] = true;
    }
  }
  for (size_t i = 0; i < done.size(); i++) {
    done_[i] = (done_[i] & done[i]);
  }
  return out_data;
}

void PosesInterpolation::SatNumInterpolationByTime() {
  std::ifstream ifs(gnss_status_path_);
  if (ifs.is_open()) {
    std::string line;
    std::vector<std::pair<double, double>> v1;
    while (getline(ifs, line)) {
      std::stringstream ss(line);
      int index;
      double timestamp;
      int solution_completed;
      int solution_status;
      int num_sats;
      if (ss >> index >> timestamp >> solution_completed >> solution_status >>
          num_sats) {
        v1.emplace_back(timestamp, num_sats);
      } else {
        std::cerr << "Parse gnss status error, content: " << line << std::endl;
      }
    }
    num_sats_ = InterpolationByTime(v1);
  } else {
    std::cerr << "Failed to load gnss best pose file: " << gnss_status_path_
              << std::endl;
    exit(1);
  }
}

void PosesInterpolation::StdDevInterpolationByTime() {
  std::ifstream ifs(gnss_best_pose_path_);
  if (ifs.is_open()) {
    std::string line;
    std::vector<std::pair<double, double>> v1, v2, v3;
    while (getline(ifs, line)) {
      std::stringstream ss(line);
      int index;
      double timestamp;
      int sol_status;
      double latitude, longitude, height_msl;
      double latitude_std_dev, longitude_std_dev, height_std_dev;
      if (ss >> index >> timestamp >> sol_status >> latitude >> longitude >>
          height_msl >> latitude_std_dev >> longitude_std_dev >>
          height_std_dev) {
        v1.emplace_back(timestamp, latitude_std_dev);
        v2.emplace_back(timestamp, longitude_std_dev);
        v3.emplace_back(timestamp, height_std_dev);
      } else {
        std::cerr << "Parse gnss best pose error, content: " << line
                  << std::endl;
        exit(1);
      }
    }
    auto r1 = InterpolationByTime(v1);
    auto r2 = InterpolationByTime(v2);
    auto r3 = InterpolationByTime(v3);
    for (size_t i = 0; i < indexes_.size(); i++) {
      std_dev_[i] = Eigen::Vector3d(r1[i], r2[i], r3[i]);
    }
  } else {
    std::cerr << "Failed to load gnss status file: " << gnss_status_path_
              << std::endl;
  }
}

void PosesInterpolation::PoseInterpolationByTime() {
  Eigen::Affine3d velodyne_extrinsic;
  std::vector<Eigen::Vector3d> input_stds;
  std::vector<Eigen::Affine3d> input_poses;
  std::vector<double> input_timestamps;
  velodyne::LoadPosesAndStds(input_poses_path_, &input_poses, &input_stds,
                             &input_timestamps);
  std::vector<std::vector<double>> odometry_poses;
  for (size_t i = 0; i < input_poses.size(); i++) {
    auto translate = input_poses[i].translation();
    auto quaternion = Eigen::Quaterniond(input_poses[i].rotation());
    odometry_poses.push_back({translate.x(), translate.y(), translate.z(),
                              quaternion.x(), quaternion.y(), quaternion.z(),
                              quaternion.w()});
  }
  std::vector<std::vector<double>> inter_pose(indexes_.size());
  for (int i = 0; i < 7; i++) {
    std::vector<std::pair<double, double>> in_data;
    for (size_t j = 0; j < odometry_poses.size(); j++) {
      in_data.emplace_back(input_timestamps[j], odometry_poses[j][i]);
    }
    const auto &inter_val = InterpolationByTime(in_data);
    for (size_t j = 0; j < inter_pose.size(); j++) {
      inter_pose[j].push_back(inter_val[j]);
    }
  }
  for (size_t i = 0; i < inter_pose.size(); i++) {
    const auto &v = inter_pose[i];
    Eigen::Translation3d trans(Eigen::Vector3d(v[0], v[1], v[2]));
    Eigen::Quaterniond quat(v[6], v[3], v[4], v[5]);
    poses_[i] = (trans * quat) * velodyne_extrinsic_;
  }
}

void PosesInterpolation::ChassisInterpolationByTime() {
  std::vector<double> timestamps, speeds, throttles, brakes, steering_percents;
  std::vector<canbus::Chassis::GearPosition> gear_locations;

  std::ifstream ifs(chassis_path_);
  if (ifs.is_open()) {
    std::string line;
    while (getline(ifs, line)) {
      std::stringstream ss(line);
      int index;
      double timestamp, speed, throttle, brake, steering_percent;
      int gear_location;
      if (ss >> index >> timestamp >> speed >> throttle >> brake >>
          steering_percent >> gear_location) {
        timestamps.push_back(timestamp);
        speeds.push_back(speed);
        throttles.push_back(throttle);
        brakes.push_back(brake);
        steering_percents.push_back(steering_percent);
        gear_locations.push_back(
            static_cast<canbus::Chassis::GearPosition>(gear_location));
      } else {
        std::cerr << "Parse chassis error, content: " << line << std::endl;
      }
    }
  } else {
    std::cerr << "Failed load to chassis file: " << chassis_path_ << std::endl;
  }
  if (timestamps.empty()) {
    std::cout << "no chassis msg found" << std::endl;
    vdm_poses_.assign(indexes_.size(), Eigen::Affine3d::Identity());
    return;
  }
  this->vdm_available_ = true;
  // calculate poses by vehicle dynamic model
  VehicleDynamicModel vdm(throttles[0], brakes[0], steering_percents[0],
                          gear_locations[0]);
  std::vector<std::vector<double>> poses(timestamps.size(),
                                         std::vector<double>(7, 0.0));
  for (size_t i = 1; i < poses.size(); i++) {
    vdm.SetSpeed(speeds[i - 1]);
    vdm.RunCommand(throttles[i - 1], brakes[i - 1], steering_percents[i - 1],
                   gear_locations[i - 1], &poses[i]);
  }

  // interpolate poses
  std::vector<std::vector<double>> inter_pose(indexes_.size());
  for (int i = 0; i < 7; i++) {
    std::vector<std::pair<double, double>> in_data;
    for (size_t j = 0; j < poses.size(); j++) {
      in_data.emplace_back(timestamps[j], poses[j][i]);
    }
    const auto &inter_val = InterpolationByTime(in_data);
    for (size_t j = 0; j < inter_pose.size(); j++) {
      inter_pose[j].push_back(inter_val[j]);
    }
  }
  for (size_t i = 0; i < inter_pose.size(); i++) {
    const auto &v = inter_pose[i];
    Eigen::Translation3d trans(Eigen::Vector3d(v[0], v[1], v[2]));
    Eigen::Quaterniond quat(v[6], v[3], v[4], v[5]);
    vdm_poses_[i] = (trans * quat) * velodyne_extrinsic_;
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
