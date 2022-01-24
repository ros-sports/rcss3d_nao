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
#include "../src/joint_pid.hpp"

TEST(TestJointPid, TestDifferentTypes)
{
  rcss3d_nao::JointPid<float, 1> pid(2, 0, 0);
  EXPECT_EQ(8, pid.update(std::array<float, 1>{1}, std::array<float, 1>{5})[0]);

  rcss3d_nao::JointPid<int, 1> pid2(2, 0, 0);
  EXPECT_EQ(8, pid2.update(std::array<int, 1>{1}, std::array<int, 1>{5})[0]);
}
