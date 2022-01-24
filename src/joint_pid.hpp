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

#ifndef JOINT_PID_HPP_
#define JOINT_PID_HPP_

#include <type_traits>
#include <array>

namespace rcss3d_nao
{

template<typename T, int S,
  std::enable_if_t<std::is_arithmetic<T>::value, bool> = true>
class JointPid
{
public:
  JointPid(float P, float I, float D)
  : P(P), I(I), D(D)
  {
  }

  std::array<T, S> update(std::array<T, S> current, std::array<T, S> expected)
  {
    std::array<T, S> output;
    for (unsigned i = 0; i < S; ++i) {
      current_error_[i] = expected[i] - current[i];
      cumulative_error_[i] += current_error_[i];
      output[i] = P * current_error_[i] +
        I * cumulative_error_[i] +
        D * (current_error_[i] - previous_error_[i]);
      previous_error_[i] = current_error_[i];
    }

    return output;
  }

private:
  std::array<T, S> current_error_{};
  std::array<T, S> previous_error_{};
  std::array<T, S> cumulative_error_{};

  float P;
  float I;
  float D;
};

}  // namespace rcss3d_nao

#endif  // JOINT_PID_HPP_
