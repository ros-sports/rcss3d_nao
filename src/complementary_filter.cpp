// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <iostream>

#include "complementary_filter.hpp"

#define ACC_WEIGHT 0.01
#define GYR_WEIGHT (1.0 - ACC_WEIGHT)

// Using implementation provided here: https://www.youtube.com/watch?v=whSw42XddsU&ab_channel=BrianDouglas
// Axes of NAO are explained here: http://doc.aldebaran.com/2-1/family/robots/inertial_robot.html

namespace rcss3d_nao
{

nao_lola_sensor_msgs::msg::Angle ComplementaryFilter::update(
  const nao_lola_sensor_msgs::msg::Accelerometer & acc,
  const nao_lola_sensor_msgs::msg::Gyroscope & gyr,
  const rclcpp::Time & time
)
{
  float angle_x_from_gyr = x_;
  float angle_y_from_gyr = y_;

  if (firstUpdate) {
    firstUpdate = false;
  } else {
    float dt = (time - prev_time_).nanoseconds() / 1e9;
    angle_x_from_gyr += gyr.x * dt;
    angle_y_from_gyr += gyr.y * dt;
  }
  prev_time_ = time;

  float angle_x_from_acc = atan2(acc.y, acc.z);
  float angle_y_from_acc = atan2(-acc.x, acc.z);

  x_ = GYR_WEIGHT * angle_x_from_gyr + ACC_WEIGHT * angle_x_from_acc;
  y_ = GYR_WEIGHT * angle_y_from_gyr + ACC_WEIGHT * angle_y_from_acc;

  return getAngle();
}

nao_lola_sensor_msgs::msg::Angle ComplementaryFilter::getAngle()
{
  nao_lola_sensor_msgs::msg::Angle angles;
  angles.x = x_;
  angles.y = y_;
  return angles;
}

}  // namespace rcss3d_nao
