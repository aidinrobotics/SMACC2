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
// Region 0, step 1 of 3. Context = SsForkJoin::orthogonal<0>.
struct StForkB : smacc2::SmaccState<StForkB, SsForkJoin::orthogonal<0>>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownOnce, OrForkA>, StForkC, SUCCESS>> reactions;

  static void staticConfigure() { configure_orthogonal<OrForkA, CbTimerCountdownOnce>(2s); }

  void onEntry() { RCLCPP_INFO(getLogger(), "[FORKJOIN] region0 StForkB onEntry (2s)"); }

  void onExit() { RCLCPP_INFO(getLogger(), "[FORKJOIN] region0 StForkB onExit -> StForkC"); }
};
}  // namespace sm_fork_join_spike
