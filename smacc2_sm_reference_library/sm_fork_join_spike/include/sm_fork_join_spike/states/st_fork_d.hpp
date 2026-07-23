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
// Region 0, final step (3 of 3). When its timer fires, it reports region-done to the
// parent (EvRegionDone) and then idles until the join tears the region down.
struct StForkD : smacc2::SmaccState<StForkD, SsForkJoin::orthogonal<0>>
{
  using SmaccState::SmaccState;

  typedef mpl::list<sc::custom_reaction<EvTimer<CbTimerCountdownOnce, OrForkA>>> reactions;

  static void staticConfigure() { configure_orthogonal<OrForkA, CbTimerCountdownOnce>(2s); }

  void onEntry() { RCLCPP_INFO(getLogger(), "[FORKJOIN] region0 StForkD onEntry (final, 2s)"); }

  void onExit() { RCLCPP_INFO(getLogger(), "[FORKJOIN] region0 StForkD onExit"); }

  sc::result react(const EvTimer<CbTimerCountdownOnce, OrForkA> &)
  {
    RCLCPP_INFO(getLogger(), "[FORKJOIN] region0 COMPLETE -> posting EvRegionDone");
    this->postEvent<smacc2_library::EvRegionDone<SsForkJoin>>();
    return this->discard_event();
  }
};
}  // namespace sm_fork_join_spike
