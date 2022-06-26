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

#include <gtest/gtest.h>
#include <optional>
#include "rcss3d_agent_msgs/msg/ball.hpp"
#include "rcss3d_agent_msgs/msg/spherical.hpp"
#include "../src/sim_to_soccer_vision_3d.hpp"
#include "../src/polar_to_point.hpp"

TEST(SimToSoccerVision3D, TestBallArrayNoBall)
{
  auto ballArray = rcss3d_nao::sim_to_soccer_vision_3d::getBallArray(std::nullopt);
  EXPECT_EQ(ballArray.balls.size(), 0u);
}

TEST(SimToSoccerVision3D, TestBallArrayOneBall)
{
  rcss3d_agent_msgs::msg::Ball ball;
  ball.center.r = 8.51;
  ball.center.phi = -0.21;
  ball.center.theta = -0.17;

  auto ballArray = rcss3d_nao::sim_to_soccer_vision_3d::getBallArray(ball);

  EXPECT_EQ(ballArray.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(ballArray.balls.size(), 1u);
  EXPECT_NEAR(ballArray.balls[0].center.x, 8.5099, 0.01);
  EXPECT_NEAR(ballArray.balls[0].center.y, -0.0312, 0.01);
  EXPECT_NEAR(ballArray.balls[0].center.z, -0.0252, 0.01);
}
