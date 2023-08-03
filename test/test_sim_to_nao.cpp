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
#include <string>
#include <utility>
#include <vector>
#include <map>
#include "../src/sim_to_nao.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"
#include "../src/angle_conversion.hpp"

using rcss3d_nao::angle_conversion::deg2rad;

void test(
  const std::vector<rcss3d_agent_msgs::msg::HingeJointPos> & simJoints,
  const std::map<int, float> & expected)
{
  auto converted = rcss3d_nao::sim_to_nao::getJointPositions(simJoints);

  for (auto const & [key, val] : expected) {
    EXPECT_EQ(converted.positions.at(key), val);
  }
}

TEST(TestSimToNao, TestJoints)
{
  std::vector<std::pair<std::string, float>> simJointsVec = {
    {"hj1", -1.0},
    {"hj2", -2.0},
    {"laj1", 1.0},
    {"laj2", 2.0},
    {"laj3", 3.0},
    {"laj4", 4.0},
    {"llj1", 5.0},
    {"llj2", 6.0},
    {"llj3", 7.0},
    {"llj4", 8.0},
    {"llj5", 9.0},
    {"llj6", 10.0},
    {"rlj2", 11.0},
    {"rlj3", 12.0},
    {"rlj4", 13.0},
    {"rlj5", 14.0},
    {"rlj6", 15.0},
    {"raj1", 16.0},
    {"raj2", 17.0},
    {"raj3", 18.0},
    {"raj4", 19.0}};

  std::vector<rcss3d_agent_msgs::msg::HingeJointPos> simJoints;
  for (auto const & [name, position] : simJointsVec) {
    rcss3d_agent_msgs::msg::HingeJointPos hjp;
    hjp.name = name;
    hjp.ax = position;
    simJoints.push_back(hjp);
  }

  std::map<int, float> expected_nao_joint_positions = {
    {nao_lola_sensor_msgs::msg::JointIndexes::HEADYAW, deg2rad(-1.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::HEADPITCH, deg2rad(2.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LSHOULDERPITCH, deg2rad(-1.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LSHOULDERROLL, deg2rad(2.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LELBOWYAW, deg2rad(3.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LELBOWROLL, deg2rad(4.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LHIPYAWPITCH, deg2rad(5.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LHIPROLL, deg2rad(6.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LHIPPITCH, deg2rad(-7.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LKNEEPITCH, deg2rad(-8.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LANKLEPITCH, deg2rad(-9.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::LANKLEROLL, deg2rad(10.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::RHIPROLL, deg2rad(11.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::RHIPPITCH, deg2rad(-12.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::RKNEEPITCH, deg2rad(-13.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::RANKLEPITCH, deg2rad(-14.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::RANKLEROLL, deg2rad(15.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::RSHOULDERPITCH, deg2rad(-16.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::RSHOULDERROLL, deg2rad(17.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::RELBOWYAW, deg2rad(18.0)},
    {nao_lola_sensor_msgs::msg::JointIndexes::RELBOWROLL, deg2rad(19.0)},

    // below should be 0 because they don't exist in simulation.
    {nao_lola_sensor_msgs::msg::JointIndexes::LWRISTYAW, 0.0},
    {nao_lola_sensor_msgs::msg::JointIndexes::RWRISTYAW, 0.0},
    {nao_lola_sensor_msgs::msg::JointIndexes::LHAND, 0.0},
    {nao_lola_sensor_msgs::msg::JointIndexes::RHAND, 0.0}};

  test(simJoints, expected_nao_joint_positions);
}

TEST(TestSimToNao, TestAccelerometer)
{
  rcss3d_agent_msgs::msg::Accelerometer acc;
  acc.x = 0.1;
  acc.y = 0.2;
  acc.z = 0.3;

  nao_lola_sensor_msgs::msg::Accelerometer convertedAccelerometer =
    rcss3d_nao::sim_to_nao::getAccelerometer(acc);
  EXPECT_NEAR(convertedAccelerometer.x, 0.2, 0.001);
  EXPECT_NEAR(convertedAccelerometer.y, -0.1, 0.001);
  EXPECT_NEAR(convertedAccelerometer.z, 0.3, 0.001);
}


TEST(TestSimToNao, TestGyroscope)
{
  rcss3d_agent_msgs::msg::GyroRate gyr;
  gyr.x = 0.1;
  gyr.y = 0.2;
  gyr.z = 0.3;

  nao_lola_sensor_msgs::msg::Gyroscope convertedGyroscope =
    rcss3d_nao::sim_to_nao::getGyroscope(gyr);
  EXPECT_NEAR(convertedGyroscope.x, 0.2 * 3.14159 / 180.0, 0.001);
  EXPECT_NEAR(convertedGyroscope.y, -0.1 * 3.14159 / 180.0, 0.001);
  EXPECT_NEAR(convertedGyroscope.z, 0.3 * 3.14159 / 180.0, 0.001);
}
