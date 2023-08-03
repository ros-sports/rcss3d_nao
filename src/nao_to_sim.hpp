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

#ifndef NAO_TO_SIM_HPP_
#define NAO_TO_SIM_HPP_

#include <vector>
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "rcss3d_agent_msgs/msg/hinge_joint_vel.hpp"
#include "nao_joint_velocities.hpp"

namespace rcss3d_nao
{
namespace nao_to_sim
{

std::vector<rcss3d_agent_msgs::msg::HingeJointVel> getHingeJointVels(
  const NaoJointVelocities & naoJoints);

}  // namespace nao_to_sim
}  // namespace rcss3d_nao

#endif  // NAO_TO_SIM_HPP_
