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
#include "rcss3d_nao/rcss3d_nao_node.hpp"
#include "rcss3d_agent/rcss3d_agent.hpp"
#include "rcss3d_agent_msgs_to_soccer_interfaces/conversion.hpp"
#include "sim_to_nao.hpp"
#include "nao_to_sim.hpp"
#include "complementary_filter.hpp"
#include "nao_joints_pid.hpp"

namespace rcss3d_nao
{

Rcss3dNaoNode::Rcss3dNaoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"rcss3d_nao_node", options},
  complementaryFilter(std::make_unique<rcss3d_nao::ComplementaryFilter>()),
  naoJointsPid(std::make_unique<rcss3d_nao::NaoJointsPid>())
{
  // Declare parameters
  RCLCPP_DEBUG(get_logger(), "Declare parameters");
  std::string model = this->declare_parameter<std::string>("model", "rsg/agent/nao/nao.rsg");
  std::string rcss3d_host = this->declare_parameter<std::string>("rcss3d/host", "127.0.0.1");
  int rcss3d_port = this->declare_parameter<int>("rcss3d/port", 3100);
  std::string team = this->declare_parameter<std::string>("team", "Anonymous");
  double theta = this->declare_parameter<double>("theta", 0.0);
  int unum = this->declare_parameter<int>("unum", 0);
  double x = this->declare_parameter<double>("x", 0.0);
  double y = this->declare_parameter<double>("y", 0.0);

  // Create Rcss3dAgent
  params = std::make_unique<rcss3d_agent::Params>(model, rcss3d_host, rcss3d_port, team, unum);
  rcss3dAgent = std::make_unique<rcss3d_agent::Rcss3dAgent>(*params);

  // Create publisher
  accelerometerPub =
    create_publisher<nao_lola_sensor_msgs::msg::Accelerometer>("sensors/accelerometer", 10);
  anglePub =
    create_publisher<nao_lola_sensor_msgs::msg::Angle>("sensors/angle", 10);
  fsrPub =
    create_publisher<nao_lola_sensor_msgs::msg::FSR>("sensors/fsr", 10);
  gyroscopePub =
    create_publisher<nao_lola_sensor_msgs::msg::Gyroscope>("sensors/gyroscope", 10);
  jointPositionsPub =
    create_publisher<nao_lola_sensor_msgs::msg::JointPositions>("sensors/joint_positions", 10);
  ballArrayPub =
    create_publisher<soccer_vision_3d_msgs::msg::BallArray>("soccer_vision_3d/balls", 10);
  goalpostArrayPub =
    create_publisher<soccer_vision_3d_msgs::msg::GoalpostArray>("soccer_vision_3d/goalposts", 10);
  markingArrayPub =
    create_publisher<soccer_vision_3d_msgs::msg::MarkingArray>("soccer_vision_3d/markings", 10);
  robotArrayPub =
    create_publisher<soccer_vision_3d_msgs::msg::RobotArray>("soccer_vision_3d/robots", 10);

  // Register callback
  rcss3dAgent->registerPerceptCallback(
    std::bind(&Rcss3dNaoNode::perceptCallback, this, std::placeholders::_1));

  // Subscriptions
  jointPositionsSub =
    create_subscription<nao_lola_command_msgs::msg::JointPositions>(
    "effectors/joint_positions", 10,
    [this](nao_lola_command_msgs::msg::JointPositions::SharedPtr cmd) {
      naoJointsPid->updateTargetFromCommand(*cmd);
    });

  beamSub =
    create_subscription<rcss3d_agent_msgs::msg::Beam>(
    "effectors/beam", 10,
    [this](rcss3d_agent_msgs::msg::Beam::SharedPtr cmd) {
      rcss3dAgent->sendBeam(*cmd);
    });

  // Beam robot
  beamToInitialPose(x, y, theta);
}

Rcss3dNaoNode::~Rcss3dNaoNode()
{
}

void Rcss3dNaoNode::perceptCallback(const rcss3d_agent_msgs::msg::Percept & percept)
{
  // Accelerometer, Gyroscope and Angle
  bool gyrFound = false;
  bool accFound = false;
  nao_lola_sensor_msgs::msg::Gyroscope gyr;
  nao_lola_sensor_msgs::msg::Accelerometer acc;

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
  nao_lola_sensor_msgs::msg::JointPositions joints = sim_to_nao::getJointPositions(
    percept.hinge_joints);
  jointPositionsPub->publish(joints);
  auto naoJointVelocities = naoJointsPid->update(joints);
  for (auto & hingeJointVel : nao_to_sim::getHingeJointVels(naoJointVelocities)) {
    rcss3dAgent->sendHingeJointVel(hingeJointVel);
  }

  // FSR
  rcss3d_agent_msgs::msg::ForceResistance leftForceResistance;
  rcss3d_agent_msgs::msg::ForceResistance rightForceResistance;
  for (auto & forceResistance : percept.force_resistances) {
    if (forceResistance.name == "lf") {
      leftForceResistance = forceResistance;
    } else if (forceResistance.name == "rf") {
      rightForceResistance = forceResistance;
    }
  }
  fsrPub->publish(sim_to_nao::getFSR(leftForceResistance, rightForceResistance));

  // Vision
  if (percept.vision.size() > 0) {
    auto & vision = percept.vision[0];

    // Ball
    auto ball = vision.ball.size() > 0 ?
      std::make_optional<rcss3d_agent_msgs::msg::Ball>(vision.ball[0]) : std::nullopt;
    ballArrayPub->publish(rcss3d_agent_msgs_to_soccer_interfaces::getBallArray(ball));

    // Goalpost
    goalpostArrayPub->publish(
      rcss3d_agent_msgs_to_soccer_interfaces::getGoalpostArray(vision.goalposts));

    // Marking
    markingArrayPub->publish(
      rcss3d_agent_msgs_to_soccer_interfaces::getMarkingArray(vision.field_lines));

    // Robot
    robotArrayPub->publish(
      rcss3d_agent_msgs_to_soccer_interfaces::getRobotArray(vision.players));
  }
}

void Rcss3dNaoNode::beamToInitialPose(double x, double y, double theta)
{
  rcss3d_agent_msgs::msg::Beam beam;
  beam.x = x;
  beam.y = y;
  beam.rot = theta;
  rcss3dAgent->sendBeam(beam);
}

}  // namespace rcss3d_nao

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rcss3d_nao::Rcss3dNaoNode)
