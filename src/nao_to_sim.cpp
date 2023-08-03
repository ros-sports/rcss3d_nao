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

#include <map>
#include <vector>
#include <utility>
#include <string>
#include <memory>
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nao_to_sim.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"

namespace rcss3d_nao
{

namespace nao_to_sim
{

static rclcpp::Logger naoToSimLogger = rclcpp::get_logger("nao_to_sim");

// LWristYaw, LHand, RWristYaw and RHand don't exist in sim so we ignore.
static const std::map<int, std::string> naoIndexToSimString = {
  {nao_lola_command_msgs::msg::JointIndexes::HEADYAW, "he1"},
  {nao_lola_command_msgs::msg::JointIndexes::HEADPITCH, "he2"},
  {nao_lola_command_msgs::msg::JointIndexes::LSHOULDERPITCH, "lae1"},
  {nao_lola_command_msgs::msg::JointIndexes::LSHOULDERROLL, "lae2"},
  {nao_lola_command_msgs::msg::JointIndexes::LELBOWYAW, "lae3"},
  {nao_lola_command_msgs::msg::JointIndexes::LELBOWROLL, "lae4"},
  {nao_lola_command_msgs::msg::JointIndexes::LHIPYAWPITCH, "lle1"},
  {nao_lola_command_msgs::msg::JointIndexes::LHIPROLL, "lle2"},
  {nao_lola_command_msgs::msg::JointIndexes::LHIPPITCH, "lle3"},
  {nao_lola_command_msgs::msg::JointIndexes::LKNEEPITCH, "lle4"},
  {nao_lola_command_msgs::msg::JointIndexes::LANKLEPITCH, "lle5"},
  {nao_lola_command_msgs::msg::JointIndexes::LANKLEROLL, "lle6"},
  {nao_lola_command_msgs::msg::JointIndexes::RHIPROLL, "rle2"},
  {nao_lola_command_msgs::msg::JointIndexes::RHIPPITCH, "rle3"},
  {nao_lola_command_msgs::msg::JointIndexes::RKNEEPITCH, "rle4"},
  {nao_lola_command_msgs::msg::JointIndexes::RANKLEPITCH, "rle5"},
  {nao_lola_command_msgs::msg::JointIndexes::RANKLEROLL, "rle6"},
  {nao_lola_command_msgs::msg::JointIndexes::RSHOULDERPITCH, "rae1"},
  {nao_lola_command_msgs::msg::JointIndexes::RSHOULDERROLL, "rae2"},
  {nao_lola_command_msgs::msg::JointIndexes::RELBOWYAW, "rae3"},
  {nao_lola_command_msgs::msg::JointIndexes::RELBOWROLL, "rae4"},
};

std::vector<int> naoJointsToInvert = {
  nao_lola_command_msgs::msg::JointIndexes::HEADPITCH,
  nao_lola_command_msgs::msg::JointIndexes::LSHOULDERPITCH,
  nao_lola_command_msgs::msg::JointIndexes::LHIPPITCH,
  nao_lola_command_msgs::msg::JointIndexes::LKNEEPITCH,
  nao_lola_command_msgs::msg::JointIndexes::LANKLEPITCH,
  nao_lola_command_msgs::msg::JointIndexes::RHIPPITCH,
  nao_lola_command_msgs::msg::JointIndexes::RKNEEPITCH,
  nao_lola_command_msgs::msg::JointIndexes::RANKLEPITCH,
  nao_lola_command_msgs::msg::JointIndexes::RSHOULDERPITCH};

std::vector<rcss3d_agent_msgs::msg::HingeJointVel> getHingeJointVels(
  const NaoJointVelocities & naoJointVelocities)
{
  std::vector<rcss3d_agent_msgs::msg::HingeJointVel> simJointVelocities;

  for (unsigned i = 0; i < naoJointVelocities.size(); ++i) {
    float naoJointVelocity = naoJointVelocities.at(i);

    // Check whether this joint exists in simulation
    std::string sim_joint_name;
    try {
      sim_joint_name = naoIndexToSimString.at(i);
    } catch (std::out_of_range &) {
      // This joint doesn't exist in simulation, ignore it
      continue;
    }

    if (std::find(
        naoJointsToInvert.begin(),
        naoJointsToInvert.end(), i) != naoJointsToInvert.end())
    {
      naoJointVelocity *= -1;
    }

    rcss3d_agent_msgs::msg::HingeJointVel simJointVelocity;
    simJointVelocity.name = sim_joint_name;
    simJointVelocity.ax = naoJointVelocity;
    simJointVelocities.push_back(simJointVelocity);

    if (i == nao_lola_command_msgs::msg::JointIndexes::LHIPYAWPITCH) {
      rcss3d_agent_msgs::msg::HingeJointVel rhyp;
      rhyp.name = "rle1";
      rhyp.ax = naoJointVelocity;
      simJointVelocities.push_back(rhyp);
    }
  }

  return simJointVelocities;
}

}  // namespace nao_to_sim

}  // namespace rcss3d_nao
