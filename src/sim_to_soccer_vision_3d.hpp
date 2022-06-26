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

#ifndef SIM_TO_SOCCER_VISION_3D_HPP_
#define SIM_TO_SOCCER_VISION_3D_HPP_

#include <optional>
#include "soccer_vision_3d_msgs/msg/ball_array.hpp"
#include "rcss3d_agent_msgs/msg/ball.hpp"

namespace rcss3d_nao
{
namespace sim_to_soccer_vision_3d
{

soccer_vision_3d_msgs::msg::BallArray getBallArray(
  const std::optional<rcss3d_agent_msgs::msg::Ball> & ball);

}  // namespace sim_to_soccer_vision_3d
}  // namespace rcss3d_nao

#endif  // SIM_TO_SOCCER_VISION_3D_HPP_
