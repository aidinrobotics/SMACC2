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

// Minimal fork-join spike:
//
//   StActivate --(1s)--> SsForkJoin{ region0: StForkB->StForkC->StForkD ,
//                                    region1: StForkF->StForkE          } --> StNext
//
// Both regions run their own multi-step state sequence concurrently. Each region's
// last state posts EvRegionDone. The parent superstate counts region-done signals
// and, once both regions reported, posts EvAllRegionsDone which drives the single
// transition to StNext. Every region uses its own orthogonal / timer resource; no
// resource is shared between the two branches.

#pragma once

#include <smacc2/smacc.hpp>

// CLIENTS
#include <cl_ros2_timer/cl_ros2_timer.hpp>

// FORK-JOIN EVENTS
#include <smacc2_library/events/fork_join_events.hpp>

// ORTHOGONALS
#include <sm_fork_join_spike/orthogonals/or_entry.hpp>
#include <sm_fork_join_spike/orthogonals/or_fork_a.hpp>
#include <sm_fork_join_spike/orthogonals/or_fork_b.hpp>

// CLIENT BEHAVIORS
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

using namespace boost;
using namespace smacc2;
using namespace cl_ros2_timer;
using namespace smacc2::default_transition_tags;
using namespace std::chrono_literals;

namespace sm_fork_join_spike
{
// STATE FORWARD DECLARATIONS
class StActivate;
class SsForkJoin;
class StForkB;
class StForkC;
class StForkD;
class StForkF;
class StForkE;
class StNext;

// STATE MACHINE
struct SmForkJoinSpike : public smacc2::SmaccStateMachineBase<SmForkJoinSpike, StActivate>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    RCLCPP_INFO(getLogger(), "[SmForkJoinSpike] onInitialize: creating independent orthogonals");
    this->createOrthogonal<OrEntry>();
    this->createOrthogonal<OrForkA>();
    this->createOrthogonal<OrForkB>();
  }
};
}  // namespace sm_fork_join_spike

// STATE INCLUDES (must be after state machine declaration)
#include <sm_fork_join_spike/states/st_activate.hpp>
#include <sm_fork_join_spike/superstates/ss_fork_join.hpp>
#include <sm_fork_join_spike/states/st_fork_b.hpp>
#include <sm_fork_join_spike/states/st_fork_c.hpp>
#include <sm_fork_join_spike/states/st_fork_d.hpp>
#include <sm_fork_join_spike/states/st_fork_f.hpp>
#include <sm_fork_join_spike/states/st_fork_e.hpp>
#include <sm_fork_join_spike/states/st_next.hpp>
