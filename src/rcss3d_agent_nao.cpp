// Copyright 2022 Kenji Brameld
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

#include <string>
#include <memory>
#include "rcss3d_agent_nao/rcss3d_agent_nao.hpp"
#include "rcss3d_agent/rcss3d_agent.hpp"
#include "./sim_to_nao.hpp"
#include "./complementary_filter.hpp"

namespace rcss3d_agent_nao
{

Rcss3dAgentNao::Rcss3dAgentNao(const rclcpp::NodeOptions & options)
: rclcpp::Node{"rcss3d_agent_nao", options}
{
  // Declare parameters
  RCLCPP_DEBUG(get_logger(), "Declare parameters");
  std::string rcss3d_host = this->declare_parameter<std::string>("rcss3d/host", "127.0.0.1");
  int rcss3d_port = this->declare_parameter<int>("rcss3d/port", 3100);
  std::string team = this->declare_parameter<std::string>("team", "Anonymous");
  int unum = this->declare_parameter<int>("unum", 0);

  // Create Rcss3dAgent
  params = std::make_unique<rcss3d_agent::Params>(rcss3d_host, rcss3d_port, team, unum);
  rcss3dAgent = std::make_unique<rcss3d_agent::Rcss3dAgent>(*params);

  // Create complementaryFilter
  complementaryFilter = std::make_unique<rcss3d_agent_nao::ComplementaryFilter>();

  // Create publisher
  accelerometerPub =
    create_publisher<nao_sensor_msgs::msg::Accelerometer>("sensors/accelerometer", 10);
  anglePub =
    create_publisher<nao_sensor_msgs::msg::Angle>("sensors/angle", 10);
  fsrPub =
    create_publisher<nao_sensor_msgs::msg::FSR>("sensors/fsr", 10);
  gyroscopePub =
    create_publisher<nao_sensor_msgs::msg::Gyroscope>("sensors/gyroscope", 10);
  jointPositionsPub =
    create_publisher<nao_sensor_msgs::msg::JointPositions>("sensors/joint_positions", 10);

  // Register callback
  rcss3dAgent->registerPerceptCallback(
    std::bind(&Rcss3dAgentNao::perceptCallback, this, std::placeholders::_1));

  // Subscriptions
  jointPositionsSub =
    create_subscription<nao_command_msgs::msg::JointPositions>(
    "effectors/joint_positions", 10,
    [this](nao_command_msgs::msg::JointPositions::SharedPtr cmd) {
      (void) cmd;
    });
}

Rcss3dAgentNao::~Rcss3dAgentNao()
{
}

void Rcss3dAgentNao::perceptCallback(const rcss3d_agent_msgs::msg::Percept & percept)
{
  // Accelerometer, Gyroscope and Angle
  bool gyrFound = false;
  bool accFound = false;
  nao_sensor_msgs::msg::Gyroscope gyr;
  nao_sensor_msgs::msg::Accelerometer acc;

  for (auto & gyroRate : percept.gyro_rates) {
    if (gyroRate.name == "torso") {
      gyrFound = true;
      gyr = sim_to_nao::getGyroscope(gyroRate);
    }
  }

  for (auto & accelerometer : percept.accelerometers) {
    if (accelerometer.name == "torso") {
      accFound = true;
      acc = sim_to_nao::getAccelerometer(accelerometer);
    }
  }

  if (gyrFound) {
    gyroscopePub->publish(gyr);
  } else {
    RCLCPP_ERROR(get_logger(), "Torso gyroscope not found in msg from simulator.");
  }

  if (accFound) {
    accelerometerPub->publish(acc);
  } else {
    RCLCPP_ERROR(get_logger(), "Torso accelerometer not found in msg from simulator.");
  }

  if (gyrFound && accFound) {
    float gameTime = percept.game_state.time;
    rclcpp::Time time(std::floor(gameTime), (gameTime - std::floor(gameTime)) * 1e9);
    anglePub->publish(complementaryFilter->update(acc, gyr, time));
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to compute torso angle.");
  }

  // Joint Position
  jointPositionsPub->publish(sim_to_nao::getJointPositions(percept.hinge_joints));
}

}  // namespace rcss3d_agent_nao
