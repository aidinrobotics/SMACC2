// Copyright 2026 Aidin Robotics
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

#pragma once

#include <smacc2/smacc.hpp>

namespace sm_fork_join_spike
{
// Region 1, step 1 of 2. Context = SsForkJoin::orthogonal<1>.
struct StForkF : smacc2::SmaccState<StForkF, SsForkJoin::orthogonal<1>>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownOnce, OrForkB>, StForkE, SUCCESS>> reactions;

  static void staticConfigure() { configure_orthogonal<OrForkB, CbTimerCountdownOnce>(3s); }

  void onEntry() { RCLCPP_INFO(getLogger(), "[FORKJOIN] region1 StForkF onEntry (3s)"); }

  void onExit() { RCLCPP_INFO(getLogger(), "[FORKJOIN] region1 StForkF onExit -> StForkE"); }
};
}  // namespace sm_fork_join_spike
