// Copyright 2025 Robosoft Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <smacc2/impl/smacc_state_machine_impl.hpp>
#include <smacc2/smacc_client_behavior.hpp>
#include <smacc2/smacc_orthogonal.hpp>
#include <smacc2/smacc_tracing/smacc_tracing.hpp>

#define statename stateMachine_->getCurrentState()->getClassName().c_str()
#define orthogonalName getName().c_str()
#define cbName clBehavior->getName().c_str()

namespace smacc2
{
void ISmaccOrthogonal::setStateMachine(ISmaccStateMachine * value)
{
  this->stateMachine_ = value;
  this->onInitialize();
  this->initializeClients();
}

void ISmaccOrthogonal::initializeClients()
{
  for (auto & c : this->clients_)
  {
    c->initialize();
  }
}

rclcpp::Node::SharedPtr ISmaccOrthogonal::getNode() { return this->stateMachine_->getNode(); }

void ISmaccOrthogonal::addClientBehavior(std::shared_ptr<smacc2::ISmaccClientBehavior> clBehavior)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (clBehavior != nullptr)
  {
    RCLCPP_DEBUG(
      getLogger(), "[Orthogonal: %s] Adding client behavior: %s.", this->getName().c_str(),
      clBehavior->getName().c_str());
    clBehavior->stateMachine_ = this->getStateMachine();
    clBehavior->currentOrthogonal = this;
    clBehavior->currentState = clBehavior->stateMachine_->getCurrentState();

    // Key by the owning (currently entering) state so parallel regions do not collide.
    clientBehaviors_[clBehavior->currentState].push_back(clBehavior);
  }
  else
  {
    RCLCPP_DEBUG(
      getLogger(), "[Orthogonal: %s] No client behaviors in this state.", this->getName().c_str());
  }
}

void ISmaccOrthogonal::onInitialize() {}

void ISmaccOrthogonal::initState(ISmaccState * state)
{
  RCLCPP_DEBUG(
    getLogger(), "[Orthogonal: %s] InitState: %s.", this->getName().c_str(),
    state->getClassName().c_str());
  // Create an (empty) behavior slot keyed by the entering state.
  clientBehaviors_[state];
}

std::string ISmaccOrthogonal::getName() const { return demangleSymbol(typeid(*this).name()); }

void ISmaccOrthogonal::runtimeConfigure(ISmaccState * state)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  auto it = clientBehaviors_.find(state);
  if (it == clientBehaviors_.end()) return;

  for (auto & clBehavior : it->second)
  {
    RCLCPP_DEBUG(
      getLogger(), "[Orthogonal: %s] runtimeConfigure(), current behavior: %s.",
      this->getName().c_str(), clBehavior->getName().c_str());

    clBehavior->runtimeConfigure();
  }
}

void ISmaccOrthogonal::onEntry(ISmaccState * state)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  auto it = clientBehaviors_.find(state);
  if (it != clientBehaviors_.end() && it->second.size() > 0)
  {
    for (auto & clBehavior : it->second)
    {
      RCLCPP_DEBUG(
        getLogger(), "[Orthogonal: %s] OnEntry, current Behavior: %s", orthogonalName, cbName);

      try
      {
        TRACEPOINT(
          smacc2_client_behavior_on_entry_start, statename, orthogonalName, cbName);
        clBehavior->executeOnEntry();
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          getLogger(),
          "[ClientBehavior: %s] Exception on Entry - continuing with next client behavior. "
          "Exception info: "
          "%s",
          cbName, e.what());
      }
      TRACEPOINT(smacc2_client_behavior_on_entry_end, statename, orthogonalName, cbName);
    }
  }
  else
  {
    RCLCPP_DEBUG(
      getLogger(), "[Orthogonal: %s] OnEntry -> empty orthogonal (no client behavior).",
      orthogonalName);
  }
}

void ISmaccOrthogonal::onExit(ISmaccState * state)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  auto it = clientBehaviors_.find(state);
  if (it != clientBehaviors_.end() && it->second.size() > 0)
  {
    for (auto & clBehavior : it->second)
    {
      RCLCPP_DEBUG(
        getLogger(), "[Orthogonal: %s] OnExit, current Behavior: %s", orthogonalName, cbName);
      try
      {
        TRACEPOINT(
          smacc2_client_behavior_on_exit_start, statename, orthogonalName, cbName);
        clBehavior->executeOnExit();
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          getLogger(),
          "[ClientBehavior: %s] Exception onExit - continuing with next client behavior. Exception "
          "info: %s",
          cbName, e.what());
      }
      TRACEPOINT(smacc2_client_behavior_on_exit_end, statename, orthogonalName, cbName);
    }
  }
  else
  {
    RCLCPP_DEBUG(getLogger(), "[Orthogonal %s] OnExit().", orthogonalName);
  }
}

void ISmaccOrthogonal::onDispose(ISmaccState * state)
{
  auto it = clientBehaviors_.find(state);
  if (it == clientBehaviors_.end()) return;

  for (auto & clBehavior : it->second)
  {
    clBehavior->dispose();
    this->getStateMachine()->disconnectSmaccSignalObject((void *)clBehavior.get());
  }

  clientBehaviors_.erase(it);
}

}  // namespace smacc2
