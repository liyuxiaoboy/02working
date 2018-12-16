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
#ifndef MODULES_LOCALIZATION_MSF_LOCAL_TOOL_POSES_INTERPOLATION_VDM
#define MODULES_LOCALIZATION_MSF_LOCAL_TOOL_POSES_INTERPOLATION_VDM

#include <Eigen/Geometry>
#include <deque>
#include <string>
#include <vector>
#include <memory>
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/control/common/interpolation_2d.h"
#include "modules/dreamview/proto/sim_control_conf.pb.h"

namespace apollo {
namespace localization {
namespace msf {

// extract code from modules/dreamview/backend/sim_control/sim_control.cc

class VehicleDynamicModel {
 public:
  VehicleDynamicModel();
  VehicleDynamicModel(double init_throttle, double init_brake,
                      double init_steering_target,
                      canbus::Chassis::GearPosition gear_location);

  void RunCommand(double throttle, double brake, double steer,
                  canbus::Chassis::GearPosition gear_location,
                  std::vector<double>* pose);
  void SetSpeed(double velocity);

 private:
  void LoadSimControlConf();
  void LongitudinalDynamicModel(double throttle, double brake, const int gear);
  void LateralDynamicModel(double steer);
  double InertiaLagElement(std::deque<double>* buffer, double input, double T,
                           double K);
  double sgn(double input);

  dreamview::SimControlConf sim_control_conf_;
  std::unique_ptr<apollo::control::Interpolation2D> sim_control_interpolation_;

  std::string sim_control_conf_file_ =
      "/apollo/modules/dreamview/conf/moonx.pb.txt";
  // The latest received control command.
  apollo::canbus::Chassis chassis_state_;

  // Whether first received reverse command.
  bool is_reverse_first_ = true;
  bool is_use_calibration_table_ = true;

  // vechile state and output.
  // state: y, y_dot, phi, phi_dot, x_dot, x_dot_dot, y_dot_dot.
  Eigen::VectorXd vehicle_state_ = Eigen::VectorXd::Zero(7);
  // output: X, Y, phi.
  Eigen::VectorXd vehicle_output_ = Eigen::VectorXd::Zero(3);

  // vehicle parameters.
  common::VehicleParam vehicle_param_;

  // the ratio between the turn of the steering wheel and the turn of the
  // wheels.
  double steer_transmission_ratio_ = 0.0;
  // the maximum turn of steer.
  double max_steer_angle_ = 0.0;
  // throttle parameters
  std::deque<double> throttle_buf_;  // a queue for lag
  // throttle parameters
  std::deque<double> brake_buf_;  // a queue for lag
  // steer parameters
  std::deque<double> steer_buf_;  // a queue for lag
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
#endif  // MODULES_LOCALIZATION_MSF_LOCAL_TOOL_POSES_INTERPOLATION_VDM
