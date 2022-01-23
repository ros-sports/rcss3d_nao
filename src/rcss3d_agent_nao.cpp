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
  for (auto & gyroRate : percept.gyro_rates) {
    if (gyroRate.name == "torso") {
      nao_sensor_msgs::msg::Gyroscope msg;
      msg.x = gyroRate.x;
      msg.y = gyroRate.y;
      msg.z = gyroRate.z;
      gyroscopePub->publish(msg);
    }
  }

  for (auto & accelerometer : percept.accelerometers) {
    if (accelerometer.name == "torso") {
      nao_sensor_msgs::msg::Accelerometer msg;
      msg.x = accelerometer.x;
      msg.y = accelerometer.y;
      msg.z = accelerometer.z;
      accelerometerPub->publish(msg);
    }
  }
}

}  // namespace rcss3d_agent_nao
