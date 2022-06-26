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

#include "sim_to_soccer_vision_3d.hpp"
#include "polar_to_point.hpp"
#include "soccer_vision_3d_msgs/msg/ball.hpp"
#include "deg2rad.hpp"

namespace rcss3d_nao
{
namespace sim_to_soccer_vision_3d
{

soccer_vision_3d_msgs::msg::BallArray getBallArray(
  const std::optional<rcss3d_agent_msgs::msg::Ball> & ball)
{
  soccer_vision_3d_msgs::msg::BallArray ballArray;
  ballArray.header.frame_id = "CameraTop_frame";
  if (ball.has_value()) {
    auto ballV = ball.value();
    auto converted = rcss3d_nao::polar_to_point(
      ballV.center.r, deg2rad(ballV.center.phi), deg2rad(ballV.center.theta));

    soccer_vision_3d_msgs::msg::Ball ball;
    ball.center = converted;
    ballArray.balls.push_back(ball);
  }
  return ballArray;
}

}  // namespace sim_to_soccer_vision_3d
}  // namespace rcss3d_nao
