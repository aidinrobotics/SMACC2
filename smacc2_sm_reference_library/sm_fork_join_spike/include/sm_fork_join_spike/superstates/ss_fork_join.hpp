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
// Parallel superstate holding two boost orthogonal regions:
//   region 0 initial = StForkB, region 1 initial = StForkF.
// The join is cooperative: each region posts EvRegionDone when it finishes its
// sequence; this superstate counts them and posts EvAllRegionsDone once both are in,
// which fires the single transition to StNext.
struct SsForkJoin
: smacc2::SmaccState<SsForkJoin, SmForkJoinSpike, mpl::list<StForkB, StForkF>>
{
  using SmaccState::SmaccState;

  int regionsDone_ = 0;

  typedef mpl::list<
    sc::custom_reaction<smacc2_library::EvRegionDone<SsForkJoin>>,
    Transition<smacc2_library::EvAllRegionsDone<SsForkJoin>, StNext, SUCCESS>>
    reactions;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[FORKJOIN] SsForkJoin onEntry: launching region0 and region1");
  }

  void onExit() { RCLCPP_INFO(getLogger(), "[FORKJOIN] SsForkJoin onExit -> StNext"); }

  sc::result react(const smacc2_library::EvRegionDone<SsForkJoin> &)
  {
    regionsDone_++;
    RCLCPP_INFO(getLogger(), "[FORKJOIN] JOIN: EvRegionDone received (count=%d/2)", regionsDone_);
    if (regionsDone_ >= 2)
    {
      RCLCPP_INFO(getLogger(), "[FORKJOIN] JOIN: both regions done -> posting EvAllRegionsDone");
      this->postEvent<smacc2_library::EvAllRegionsDone<SsForkJoin>>();
    }
    return this->discard_event();
  }
};
}  // namespace sm_fork_join_spike
