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

#include <boost/filesystem.hpp>
#include "gflags/gflags.h"
#include "modules/localization/msf/local_tool/map_creation/poses_interpolation/poses_interpolation.h"

DEFINE_string(input_poses_path, "", "provide input poses path");
DEFINE_string(ref_timestamps_path, "", "provide reference timestamp path");
DEFINE_string(extrinsic_path, "", "provide velodyne extrinsic path");
DEFINE_string(gnss_best_pose_path, "", "provide gnss best pose path");
DEFINE_string(gnss_status_path, "", "provide gnss status path");
DEFINE_string(chassis_path, "", "provide chassis path");
DEFINE_string(output_poses_path, "", "provide output poses path");

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::localization::msf::PosesInterpolation pose_interpolation;
  pose_interpolation.DoInterpolation(
      FLAGS_input_poses_path, FLAGS_ref_timestamps_path,
      FLAGS_output_poses_path, FLAGS_extrinsic_path, FLAGS_gnss_best_pose_path,
      FLAGS_gnss_status_path, FLAGS_chassis_path);
  return 0;
}
