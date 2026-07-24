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
#include <smacc2_library/events/fork_join_events.hpp>

namespace sm_fork_join_spike
{
// Region 1, final step (2 of 2). Slower than region 0 so the two regions finish at
// clearly different times, proving the join waits for the slower branch.
struct StForkE : smacc2::SmaccState<StForkE, SsForkJoin::orthogonal<1>>
{
  using SmaccState::SmaccState;

  typedef mpl::list<sc::custom_reaction<EvTimer<CbTimerCountdownOnce, OrForkB>>> reactions;

  static void staticConfigure() { configure_orthogonal<OrForkB, CbTimerCountdownOnce>(4s); }

  void onEntry() { RCLCPP_INFO(getLogger(), "[FORKJOIN] region1 StForkE onEntry (final, 4s)"); }

  void onExit() { RCLCPP_INFO(getLogger(), "[FORKJOIN] region1 StForkE onExit"); }

  sc::result react(const EvTimer<CbTimerCountdownOnce, OrForkB> &)
  {
    RCLCPP_INFO(getLogger(), "[FORKJOIN] region1 COMPLETE -> posting EvRegionDone");
    this->postEvent<smacc2_library::EvRegionDone<SsForkJoin>>();
    return this->discard_event();
  }
};
}  // namespace sm_fork_join_spike
