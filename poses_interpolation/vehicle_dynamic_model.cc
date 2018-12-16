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
#include "modules/localization/msf/local_tool/map_creation/poses_interpolation/vehicle_dynamic_model.h"
#include <utility>
#include "modules/common/configs/config_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/file.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace localization {
namespace msf {
VehicleDynamicModel::VehicleDynamicModel() {
  this->sim_control_conf_file_ = FLAGS_sim_control_conf_file;
  LoadSimControlConf();
}

VehicleDynamicModel::VehicleDynamicModel(
    double init_throttle, double init_brake, double init_steering_target,
    canbus::Chassis::GearPosition gear_location)
    : VehicleDynamicModel() {
  std::fill(throttle_buf_.begin(), throttle_buf_.end(), init_throttle);
  std::fill(brake_buf_.begin(), brake_buf_.end(), init_brake);
  double steer =
      init_steering_target * max_steer_angle_ / steer_transmission_ratio_ / 100;
  std::fill(steer_buf_.begin(), steer_buf_.end(), steer);
  chassis_state_.set_gear_location(gear_location);
  if (gear_location == canbus::Chassis::GEAR_REVERSE) {
    is_reverse_first_ = false;
  }
}

void VehicleDynamicModel::SetSpeed(double velocity) {
  vehicle_state_[4] = velocity;
}

void VehicleDynamicModel::RunCommand(
    double throttle, double brake, double steering_target,
    canbus::Chassis::GearPosition gear_location, std::vector<double>* pose) {
  if (throttle < 0 || throttle > 100 || brake < 0 || brake > 100 ||
      steering_target < -100 || steering_target > 100) {
    AERROR << "Control command error!";
  }
  double steer =
      steering_target * max_steer_angle_ / steer_transmission_ratio_ / 100;

  double throttle_K = sim_control_conf_.throttle_proportion();
  double throttle_T = sim_control_conf_.throttle_period();
  throttle =
      InertiaLagElement(&throttle_buf_, throttle, throttle_T, throttle_K);

  // inertia element for brake
  double brake_K = sim_control_conf_.brake_proportion();
  double brake_T = sim_control_conf_.brake_period();
  brake = InertiaLagElement(&brake_buf_, brake, brake_T, brake_K);

  // inertia element for steer
  double steer_K = sim_control_conf_.steer_proportion();
  double steer_T = sim_control_conf_.steer_period();
  steer = InertiaLagElement(&steer_buf_, steer, steer_T, steer_K);
  if (chassis_state_.gear_location() != gear_location) {
    if (vehicle_state_[4] < 0.01 && brake > 0) {
      chassis_state_.set_gear_location(gear_location);
    } else {
      AERROR << "Can not change gear location!" << std::endl;
    }
  }

  // judge if car move reverse
  if (chassis_state_.gear_location() == canbus::Chassis::GEAR_REVERSE &&
      is_reverse_first_) {
    vehicle_state_[2] = M_PI - vehicle_state_[2];
    is_reverse_first_ = false;
  }
  // if gear is drive or reverse, work dynamic model
  if (chassis_state_.gear_location() == canbus::Chassis::GEAR_DRIVE ||
      chassis_state_.gear_location() == canbus::Chassis::GEAR_REVERSE) {
    // longitudinal dynamic model
    LongitudinalDynamicModel(throttle, brake, chassis_state_.gear_location());

    // lateral dynamic model
    LateralDynamicModel(steer);

    // calculate vehicle output: X, Y, phi
    double dt = sim_control_conf_.dt();
    if (chassis_state_.gear_location() == canbus::Chassis::GEAR_DRIVE) {
      vehicle_output_[0] += (vehicle_state_[4] * cos(vehicle_state_[2]) -
                             vehicle_state_[1] * sin(vehicle_state_[2])) *
                            dt;
      vehicle_output_[1] += (vehicle_state_[4] * sin(vehicle_state_[2]) +
                             vehicle_state_[1] * cos(vehicle_state_[2])) *
                            dt;
      is_reverse_first_ = true;
    } else if (chassis_state_.gear_location() ==
               canbus::Chassis::GEAR_REVERSE) {
      vehicle_output_[0] +=
          (-vehicle_state_[4] * sin(vehicle_state_[2] - M_PI / 2) +
           vehicle_state_[1] * cos(vehicle_state_[2] - M_PI / 2)) *
          dt;
      vehicle_output_[1] +=
          (-vehicle_state_[4] * cos(vehicle_state_[2] - M_PI / 2) -
           vehicle_state_[1] * sin(vehicle_state_[2] - M_PI / 2)) *
          dt;
    }
    vehicle_output_[2] = vehicle_state_[2];
  }
  // set control command after inertia and lag element
  chassis_state_.set_throttle_percentage(throttle);
  chassis_state_.set_brake_percentage(brake);
  chassis_state_.set_steering_percentage(steer * steer_transmission_ratio_ *
                                         100 / max_steer_angle_);

  Eigen::Translation3d trans(
      Eigen::Vector3d(vehicle_output_[0], vehicle_output_[1], 0.0));
  Eigen::Quaterniond quat =
      apollo::common::math::HeadingToQuaternion<double>(vehicle_output_[2]);
  *pose = std::vector<double>{
      vehicle_output_[0], vehicle_output_[1], 0.0,     quat.x(),
      quat.y(),           quat.z(),           quat.w()};
}

void VehicleDynamicModel::LongitudinalDynamicModel(double throttle,
                                                   double brake,
                                                   const int gear) {
  double dt = sim_control_conf_.dt();
  double g = sim_control_conf_.g();
  double m = sim_control_conf_.mass();
  double rw = sim_control_conf_.undeformed_radius();
  double rstat = sim_control_conf_.static_radius();
  double reff = sin(acos(rstat / rw)) / acos(rstat / rw) * rw;
  double rho = sim_control_conf_.air_density();
  double cd = sim_control_conf_.frag_coefficient();
  double af = sim_control_conf_.frontal_area();
  double vwind = sim_control_conf_.wind_velocity();
  double f0 = sim_control_conf_.roll_resistance();
  double vthre = sim_control_conf_.threshold_velocity();
  double slope = sim_control_conf_.slope();
  double speed = vehicle_state_[4];
  double command = throttle - brake;
  double vx_dot = 0.0;
  if (is_use_calibration_table_) {
    vx_dot =
        sim_control_interpolation_->Interpolate(std::make_pair(speed, command));
  } else {
    if (gear == canbus::Chassis::GEAR_DRIVE) {
      double faero = 0.5 * rho * cd * af * pow((vehicle_state_[4] + vwind), 2) *
                     sgn(vehicle_state_[4] + vwind);
      double rx = f0 * tanh(4 * vehicle_state_[4] / vthre) * m * g * cos(slope);
      vx_dot =
          (throttle / reff - brake / reff - faero - rx) / m - g * sin(slope);
    } else if (gear == canbus::Chassis::GEAR_REVERSE) {
      double faero = 0.5 * rho * cd * af * pow((vehicle_state_[4] - vwind), 2) *
                     sgn(vehicle_state_[4] - vwind);
      double rx = f0 * tanh(4 * vehicle_state_[4] / vthre) * m * g * cos(slope);
      vx_dot =
          (throttle / reff - brake / reff - faero - rx) / m + g * sin(slope);
    }
  }
  vehicle_state_[4] += vx_dot * dt;
  vehicle_state_[5] = vx_dot;

  // velocity limit
  if (vehicle_state_[4] < 1e-6) {
    vehicle_state_[4] = 1e-6;
  }
}

void VehicleDynamicModel::LateralDynamicModel(double steer) {
  double dt = sim_control_conf_.dt();
  double m = sim_control_conf_.mass();
  double lf = sim_control_conf_.front_length();
  double lr = sim_control_conf_.rear_length();
  double iz = m / 2 * (lf * lf + lr * lr);
  double caf = sim_control_conf_.front_cornering_stiffness();
  double car = sim_control_conf_.rear_cornering_stiffness();
  // calculate continuous matrix A and B
  Eigen::MatrixXd A(4, 4);
  A << 0, 1, 0, 0, 0, -(2 * caf + 2 * car) / (m * vehicle_state_[4]), 0,
      -vehicle_state_[4] -
          (2 * caf * lf - 2 * car * lr) / (m * vehicle_state_[4]),
      0, 0, 0, 1, 0, -(2 * caf * lf - 2 * car * lr) / (iz * vehicle_state_[4]),
      0, -(2 * caf * lf * lf + 2 * car * lr * lr) / (iz * vehicle_state_[4]);
  Eigen::VectorXd B(4);
  B << 0, 2 * caf / m, 0, 2 * lf * caf / iz;
  // calculate discrete matrix G and H
  Eigen::MatrixXd G =
      (Eigen::MatrixXd::Identity(4, 4) - dt * 0.5 * A).inverse() *
      (Eigen::MatrixXd::Identity(4, 4) + dt * 0.5 * A);
  Eigen::MatrixXd H =
      dt * (Eigen::MatrixXd::Identity(4, 4) - dt * 0.5 * A).inverse() * B;
  // calculate next vehicle state
  vehicle_state_.head(4) = G * vehicle_state_.head(4) + H * steer;
  // calculate vy_dot
  Eigen::VectorXd lat_state = A * vehicle_state_.head(4) + B * steer;
  vehicle_state_[6] = lat_state[1];
}

double VehicleDynamicModel::InertiaLagElement(std::deque<double>* buffer,
                                              double input, double T,
                                              double K) {
  double dt = sim_control_conf_.dt();
  // first order inertia
  double y_old = buffer->back();
  double y = (T / (T + dt)) * y_old + (K * dt / (T + dt)) * input;

  // pure lag
  double output = buffer->front();
  buffer->pop_front();
  buffer->push_back(y);
  return output;
}

void VehicleDynamicModel::LoadSimControlConf() {
  // get vehicle_param for steer
  const auto& vehicle_param_ =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  steer_transmission_ratio_ = vehicle_param_.steer_ratio();
  max_steer_angle_ = vehicle_param_.max_steer_angle();

  // load sim_control_conf file
  apollo::common::util::GetProtoFromFile(sim_control_conf_file_,
                                         &sim_control_conf_);

  // build longitudinal calibration table
  const auto& control_table = sim_control_conf_.lon_calibration_table();
  apollo::control::Interpolation2D::DataType xyz;
  for (const auto& calibration : control_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(), calibration.command(),
                                  calibration.acceleration()));
  }
  sim_control_interpolation_.reset(new apollo::control::Interpolation2D);
  CHECK(sim_control_interpolation_->Init(xyz))
      << "Fail to load sim_control calibration table";

  // load throttle, brake and steer parameters
  double dt = sim_control_conf_.dt();
  double throttle_lag = sim_control_conf_.throttle_lag();
  throttle_buf_.assign(static_cast<int>(throttle_lag / dt), 0.0);
  double brake_lag = sim_control_conf_.brake_lag();
  brake_buf_.assign(static_cast<int>(brake_lag / dt), 0.0);
  double steer_lag = sim_control_conf_.steer_lag();
  steer_buf_.assign(static_cast<int>(steer_lag / dt), 0.0);

  chassis_state_.set_throttle_percentage(0.0);
  chassis_state_.set_brake_percentage(0.0);
  chassis_state_.set_steering_percentage(0.0);
  chassis_state_.set_driving_mode(sim_control_conf_.driving_mode());
  chassis_state_.set_gear_location(sim_control_conf_.gear_location());
}

double VehicleDynamicModel::sgn(double input) {
  if (input > 0) {
    return 1;
  } else if (input < 0) {
    return -1;
  } else {
    return 0;
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
