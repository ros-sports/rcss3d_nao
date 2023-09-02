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

#ifndef RCSS3D_NAO__RCSS3D_NAO_NODE_HPP_
#define RCSS3D_NAO__RCSS3D_NAO_NODE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/node.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_sensor_msgs/msg/accelerometer.hpp"
#include "nao_lola_sensor_msgs/msg/angle.hpp"
#include "nao_lola_sensor_msgs/msg/fsr.hpp"
#include "nao_lola_sensor_msgs/msg/gyroscope.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "rcss3d_agent_msgs/msg/percept.hpp"
#include "rcss3d_agent_msgs/msg/beam.hpp"
#include "soccer_vision_3d_msgs/msg/ball_array.hpp"
#include "soccer_vision_3d_msgs/msg/goalpost_array.hpp"
#include "soccer_vision_3d_msgs/msg/marking_array.hpp"
#include "soccer_vision_3d_msgs/msg/robot_array.hpp"

// Forward Declaration
namespace rcss3d_agent
{
class Rcss3dAgent;
class Params;
}
namespace rcss3d_nao
{
class ComplementaryFilter;
class NaoJointsPid;
}

namespace rcss3d_nao
{

class Rcss3dNaoNode : public rclcpp::Node
{
public:
  explicit Rcss3dNaoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~Rcss3dNaoNode();

private:
  std::unique_ptr<rcss3d_agent::Params> params;
  std::unique_ptr<rcss3d_agent::Rcss3dAgent> rcss3dAgent;
  std::unique_ptr<rcss3d_nao::ComplementaryFilter> complementaryFilter;
  std::unique_ptr<rcss3d_nao::NaoJointsPid> naoJointsPid;

  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Accelerometer>::SharedPtr accelerometerPub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Angle>::SharedPtr anglePub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::FSR>::SharedPtr fsrPub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::Gyroscope>::SharedPtr gyroscopePub;
  rclcpp::Publisher<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr jointPositionsPub;
  rclcpp::Publisher<soccer_vision_3d_msgs::msg::BallArray>::SharedPtr ballArrayPub;
  rclcpp::Publisher<soccer_vision_3d_msgs::msg::GoalpostArray>::SharedPtr goalpostArrayPub;
  rclcpp::Publisher<soccer_vision_3d_msgs::msg::MarkingArray>::SharedPtr markingArrayPub;
  rclcpp::Publisher<soccer_vision_3d_msgs::msg::RobotArray>::SharedPtr robotArrayPub;

  rclcpp::Subscription<nao_lola_command_msgs::msg::JointPositions>::SharedPtr jointPositionsSub;
  rclcpp::Subscription<rcss3d_agent_msgs::msg::Beam>::SharedPtr beamSub;

  void perceptCallback(const rcss3d_agent_msgs::msg::Percept & percept);
  void beamToInitialPose(double x, double y, double theta);
};

}  // namespace rcss3d_nao

#endif  // RCSS3D_NAO__RCSS3D_NAO_NODE_HPP_
