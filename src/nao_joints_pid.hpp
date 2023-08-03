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

#ifndef NAO_JOINTS_PID_HPP_
#define NAO_JOINTS_PID_HPP_

#include <vector>
#include "joint_pid.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_joint_velocities.hpp"

namespace rcss3d_nao
{

class NaoJointsPid
{
public:
  NaoJointsPid();
  void updateTargetFromCommand(const nao_lola_command_msgs::msg::JointPositions & target);
  NaoJointVelocities update(const nao_lola_sensor_msgs::msg::JointPositions & current);

private:
  JointPid<float, nao_lola_command_msgs::msg::JointIndexes::NUMJOINTS> jointPid;
  nao_lola_sensor_msgs::msg::JointPositions target;
};

}  // namespace rcss3d_nao

#endif  // NAO_JOINTS_PID_HPP_
