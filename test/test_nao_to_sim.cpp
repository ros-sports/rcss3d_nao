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

#include <gtest/gtest.h>
#include <map>
#include <utility>
#include <vector>
#include <algorithm>
#include <string>
#include "../src/nao_to_sim.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"

void test_contains(
  const std::vector<rcss3d_agent_msgs::msg::HingeJointVel> & list,
  const rcss3d_agent_msgs::msg::HingeJointVel & elementToFind)
{
  float velocity = 0;
  bool found = false;
  for (auto & element : list) {
    if (element.name == elementToFind.name) {
      found = true;
      velocity = element.ax;
      break;
    }
  }

  ASSERT_TRUE(found);
  EXPECT_NEAR(elementToFind.ax, velocity, 0.00001);
}


TEST(TestJointsNaoToSim, TestOneJoint)
{
  NaoJointVelocities naoJointVelocities{};
  naoJointVelocities.at(nao_lola_command_msgs::msg::JointIndexes::LHIPROLL) = 0.4;

  auto convertedVels = rcss3d_nao::nao_to_sim::getHingeJointVels(naoJointVelocities);

  rcss3d_agent_msgs::msg::HingeJointVel j;
  j.name = "lle2";
  j.ax = 0.4;
  test_contains(convertedVels, j);
}

TEST(TestJointsNaoToSim, TestOneJointInversion)
{
  NaoJointVelocities naoJointVelocities{};
  naoJointVelocities.at(nao_lola_command_msgs::msg::JointIndexes::LHIPPITCH) = 0.6;

  auto convertedVels = rcss3d_nao::nao_to_sim::getHingeJointVels(naoJointVelocities);

  rcss3d_agent_msgs::msg::HingeJointVel j;
  j.name = "lle3";
  j.ax = -0.6;
  test_contains(convertedVels, j);
}


TEST(TestJointsNaoToSim, Test)
{
  // The order of joints can be in any order.
  // In this case, headpitch is missing and headyaw is at end of vector.
  std::vector<std::pair<int, float>> naoJointVelocitiesVec = {
    {nao_lola_command_msgs::msg::JointIndexes::HEADPITCH, 0},
    {nao_lola_command_msgs::msg::JointIndexes::LSHOULDERPITCH, 0.01},
    {nao_lola_command_msgs::msg::JointIndexes::LSHOULDERROLL, 0.02},
    {nao_lola_command_msgs::msg::JointIndexes::LELBOWYAW, 0.03},
    {nao_lola_command_msgs::msg::JointIndexes::LELBOWROLL, 0.04},
    {nao_lola_command_msgs::msg::JointIndexes::LWRISTYAW, 0.05},
    {nao_lola_command_msgs::msg::JointIndexes::LHIPYAWPITCH, 0.06},
    {nao_lola_command_msgs::msg::JointIndexes::LHIPROLL, 0.07},
    {nao_lola_command_msgs::msg::JointIndexes::LHIPPITCH, 0.08},
    {nao_lola_command_msgs::msg::JointIndexes::LKNEEPITCH, 0.09},
    {nao_lola_command_msgs::msg::JointIndexes::LANKLEPITCH, 0.10},
    {nao_lola_command_msgs::msg::JointIndexes::LANKLEROLL, 0.11},
    {nao_lola_command_msgs::msg::JointIndexes::RHIPROLL, 0.12},
    {nao_lola_command_msgs::msg::JointIndexes::RHIPPITCH, 0.13},
    {nao_lola_command_msgs::msg::JointIndexes::RKNEEPITCH, 0.14},
    {nao_lola_command_msgs::msg::JointIndexes::RANKLEPITCH, 0.15},
    {nao_lola_command_msgs::msg::JointIndexes::RANKLEROLL, 0.16},
    {nao_lola_command_msgs::msg::JointIndexes::RSHOULDERPITCH, 0.17},
    {nao_lola_command_msgs::msg::JointIndexes::RSHOULDERROLL, 0.18},
    {nao_lola_command_msgs::msg::JointIndexes::RELBOWYAW, 0.19},
    {nao_lola_command_msgs::msg::JointIndexes::RELBOWROLL, 0.20},
    {nao_lola_command_msgs::msg::JointIndexes::RWRISTYAW, 0.21},
    {nao_lola_command_msgs::msg::JointIndexes::LHAND, 0.22},
    {nao_lola_command_msgs::msg::JointIndexes::RHAND, 0.23},
    {nao_lola_command_msgs::msg::JointIndexes::HEADYAW, -0.01}};

  std::map<std::string, float> expectedHingeJointVelsMap = {
    {"he1", -0.01},
    {"he2", 0},
    {"lae1", -0.01},
    {"lae2", 0.02},
    {"lae3", 0.03},
    {"lae4", 0.04},
    {"lle1", 0.06},
    {"lle2", 0.07},
    {"lle3", -0.08},
    {"lle4", -0.09},
    {"lle5", -0.10},
    {"lle6", 0.11},
    {"rle1", 0.06},
    {"rle2", 0.12},
    {"rle3", -0.13},
    {"rle4", -0.14},
    {"rle5", -0.15},
    {"rle6", 0.16},
    {"rae1", -0.17},
    {"rae2", 0.18},
    {"rae3", 0.19},
    {"rae4", 0.20},
  };

  NaoJointVelocities naoJointVelocities{};
  for (auto j : naoJointVelocitiesVec) {
    naoJointVelocities.at(j.first) = j.second;
  }
  auto convertedVels = rcss3d_nao::nao_to_sim::getHingeJointVels(naoJointVelocities);

  for (auto const & [name, velocity] : expectedHingeJointVelsMap) {
    rcss3d_agent_msgs::msg::HingeJointVel j;
    j.name = name;
    j.ax = velocity;
    test_contains(convertedVels, j);
  }
}
