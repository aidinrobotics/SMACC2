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

#include <boost/statechart/event.hpp>
#include <boost/statechart/result.hpp>
#include <smacc2/common.hpp>

namespace smacc2
{
// Guarded terminal reaction for a parallel orthogonal region.
//
// When `TriggerEvent` arrives at the region's terminal state AND it is owned by this
// state's own region, this reaction posts `TRegionDoneEvent` (the parallel parent's
// join signal) and consumes the trigger. If the trigger belongs to a sibling parallel
// region it is forwarded so the region that actually owns it receives it -- the same
// ownership guard used by smacc2::Transition, applied here so a terminal region state
// cannot steal a sibling region's completion event.
//
// This lets the code generator express "region finished -> signal parent" as a single
// reaction list entry, instead of a hand-written custom_reaction whose body would have
// to repeat the guard. The core owns the guard; the generator only names the trigger
// event and the join event to post.
template <class TriggerEvent, class TRegionDoneEvent>
class RegionCompletion
{
public:
  template <class State, class EventBase, class IdType>
  static boost::statechart::detail::reaction_result react(
    State & stt, const EventBase & evt, const IdType & eventType)
  {
    // Only handle our specific trigger event type; forward everything else so other
    // reactions in the list (and outer states) still see it.
    if (eventType != TriggerEvent::static_type())
    {
      return boost::statechart::detail::do_forward_event;
    }

    // Ownership guard: only the region that owns this completion event handles it.
    const auto * ownedEvent = dynamic_cast<const smacc2::SmaccEventBase *>(&evt);
    if (
      ownedEvent != nullptr &&
      stt.getStateMachine().eventOwnerInDifferentBranch(ownedEvent->eventOwner_, &stt))
    {
      return boost::statechart::detail::do_forward_event;
    }

    stt.template postEvent<TRegionDoneEvent>();
    return boost::statechart::detail::do_discard_event;
  }
};
}  // namespace smacc2
