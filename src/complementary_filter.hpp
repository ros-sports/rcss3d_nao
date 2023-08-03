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

#ifndef COMPLEMENTARY_FILTER_HPP_
#define COMPLEMENTARY_FILTER_HPP_

#include "nao_lola_sensor_msgs/msg/angle.hpp"
#include "nao_lola_sensor_msgs/msg/accelerometer.hpp"
#include "nao_lola_sensor_msgs/msg/gyroscope.hpp"
#include "rclcpp/time.hpp"

/*
 *  The simulator does not provide an AngleX or AngleY sensor. Luckily,
 *  we can calculate this information ourselves using the gyroscope and
 *  accelerometer information. This class does this to provide an estimation
 *  of such a sensor.
 */

namespace rcss3d_nao
{

class ComplementaryFilter
{
public:
  ComplementaryFilter(float initial_x, float initial_y)
  : x_(initial_x), y_(initial_y)
  {
  }

  ComplementaryFilter()
  : x_(0.0f), y_(0.0f)
  {
  }

  nao_lola_sensor_msgs::msg::Angle update(
    const nao_lola_sensor_msgs::msg::Accelerometer & acc,
    const nao_lola_sensor_msgs::msg::Gyroscope & gyr,
    const rclcpp::Time & time);

private:
  float x_;
  float y_;

  bool firstUpdate = true;
  rclcpp::Time prev_time_;

  nao_lola_sensor_msgs::msg::Angle getAngle();
};

}  // namespace rcss3d_nao

#endif  // COMPLEMENTARY_FILTER_HPP_
