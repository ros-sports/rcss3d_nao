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

#include "nao_joints_pid.hpp"

#define P 28.0
#define I 0.0
#define D 0.0

namespace rcss3d_nao
{

NaoJointsPid::NaoJointsPid()
: jointPid(P, I, D)
{}

void NaoJointsPid::updateTargetFromCommand(
  const nao_lola_command_msgs::msg::JointPositions & target)
{
  for (unsigned i = 0; i < target.indexes.size(); ++i) {
    int index = target.indexes.at(i);
    float position = target.positions.at(i);
    this->target.positions.at(index) = position;
  }
}

NaoJointVelocities NaoJointsPid::update(const nao_lola_sensor_msgs::msg::JointPositions & current)
{
  NaoJointVelocities out = jointPid.update(current.positions, target.positions);
  return out;
}

}  // namespace rcss3d_nao
