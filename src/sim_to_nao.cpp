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

#include <string>
#include <map>
#include <vector>
#include <utility>
#include "sim_to_nao.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"
#include "angle_conversion.hpp"

namespace rcss3d_nao
{
namespace sim_to_nao
{

// Ignore rlj1 (RHipYawPitch) because its not an actual joint on the Nao
std::map<std::string, int> name_sim_to_nao = {
  {"hj1", nao_lola_sensor_msgs::msg::JointIndexes::HEADYAW},
  {"hj2", nao_lola_sensor_msgs::msg::JointIndexes::HEADPITCH},
  {"laj1", nao_lola_sensor_msgs::msg::JointIndexes::LSHOULDERPITCH},
  {"laj2", nao_lola_sensor_msgs::msg::JointIndexes::LSHOULDERROLL},
  {"laj3", nao_lola_sensor_msgs::msg::JointIndexes::LELBOWYAW},
  {"laj4", nao_lola_sensor_msgs::msg::JointIndexes::LELBOWROLL},
  {"llj1", nao_lola_sensor_msgs::msg::JointIndexes::LHIPYAWPITCH},
  {"llj2", nao_lola_sensor_msgs::msg::JointIndexes::LHIPROLL},
  {"llj3", nao_lola_sensor_msgs::msg::JointIndexes::LHIPPITCH},
  {"llj4", nao_lola_sensor_msgs::msg::JointIndexes::LKNEEPITCH},
  {"llj5", nao_lola_sensor_msgs::msg::JointIndexes::LANKLEPITCH},
  {"llj6", nao_lola_sensor_msgs::msg::JointIndexes::LANKLEROLL},
  {"rlj2", nao_lola_sensor_msgs::msg::JointIndexes::RHIPROLL},
  {"rlj3", nao_lola_sensor_msgs::msg::JointIndexes::RHIPPITCH},
  {"rlj4", nao_lola_sensor_msgs::msg::JointIndexes::RKNEEPITCH},
  {"rlj5", nao_lola_sensor_msgs::msg::JointIndexes::RANKLEPITCH},
  {"rlj6", nao_lola_sensor_msgs::msg::JointIndexes::RANKLEROLL},
  {"raj1", nao_lola_sensor_msgs::msg::JointIndexes::RSHOULDERPITCH},
  {"raj2", nao_lola_sensor_msgs::msg::JointIndexes::RSHOULDERROLL},
  {"raj3", nao_lola_sensor_msgs::msg::JointIndexes::RELBOWYAW},
  {"raj4", nao_lola_sensor_msgs::msg::JointIndexes::RELBOWROLL}};

std::vector<int> naoJointsToInvert = {
  nao_lola_sensor_msgs::msg::JointIndexes::HEADPITCH,
  nao_lola_sensor_msgs::msg::JointIndexes::LSHOULDERPITCH,
  nao_lola_sensor_msgs::msg::JointIndexes::LHIPPITCH,
  nao_lola_sensor_msgs::msg::JointIndexes::LKNEEPITCH,
  nao_lola_sensor_msgs::msg::JointIndexes::LANKLEPITCH,
  nao_lola_sensor_msgs::msg::JointIndexes::RHIPPITCH,
  nao_lola_sensor_msgs::msg::JointIndexes::RKNEEPITCH,
  nao_lola_sensor_msgs::msg::JointIndexes::RANKLEPITCH,
  nao_lola_sensor_msgs::msg::JointIndexes::RSHOULDERPITCH};

nao_lola_sensor_msgs::msg::JointPositions getJointPositions(
  const std::vector<rcss3d_agent_msgs::msg::HingeJointPos> & simJoints)
{
  auto naoJoints = nao_lola_sensor_msgs::msg::JointPositions{};

  for (auto & sim_joint : simJoints) {
    auto it = name_sim_to_nao.find(sim_joint.name);

    if (it != name_sim_to_nao.end()) {
      float joint_index = it->second;

      auto sim_joint_position = sim_joint.ax;

      if (std::find(
          naoJointsToInvert.begin(),
          naoJointsToInvert.end(),
          joint_index) != naoJointsToInvert.end())
      {
        sim_joint_position *= -1;
      }

      naoJoints.positions[joint_index] =
        rcss3d_nao::angle_conversion::deg2rad(sim_joint_position);
    }
  }
  return naoJoints;
}

nao_lola_sensor_msgs::msg::Accelerometer getAccelerometer(
  const rcss3d_agent_msgs::msg::Accelerometer & accelerometer)
{
  nao_lola_sensor_msgs::msg::Accelerometer acc;
  acc.x = accelerometer.y;
  acc.y = -accelerometer.x;
  acc.z = accelerometer.z;
  return acc;
}

nao_lola_sensor_msgs::msg::Gyroscope getGyroscope(
  const rcss3d_agent_msgs::msg::GyroRate & gyroRate)
{
  nao_lola_sensor_msgs::msg::Gyroscope gyr;
  gyr.x = angle_conversion::deg2rad(gyroRate.y);
  gyr.y = -angle_conversion::deg2rad(gyroRate.x);
  gyr.z = angle_conversion::deg2rad(gyroRate.z);
  return gyr;
}

nao_lola_sensor_msgs::msg::FSR getFSR(
  const rcss3d_agent_msgs::msg::ForceResistance & leftForceResistance,
  const rcss3d_agent_msgs::msg::ForceResistance & rightForceResistance)
{
  // Simply copy the force Z vector into front/back, left/right sensors
  // Future improvement to get more accurate readings would be good, but this
  // is good enough for now to simply figure out which foot is experiencing
  // more of the robot's weight.
  nao_lola_sensor_msgs::msg::FSR fsr;
  fsr.l_foot_front_left = leftForceResistance.fz;
  fsr.l_foot_front_right = leftForceResistance.fz;
  fsr.l_foot_back_left = leftForceResistance.fz;
  fsr.l_foot_back_right = leftForceResistance.fz;
  fsr.r_foot_front_left = rightForceResistance.fz;
  fsr.r_foot_front_right = rightForceResistance.fz;
  fsr.r_foot_back_left = rightForceResistance.fz;
  fsr.r_foot_back_right = rightForceResistance.fz;
  return fsr;
}

}  // namespace sim_to_nao
}  // namespace rcss3d_nao
