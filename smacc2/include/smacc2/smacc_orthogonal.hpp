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

#pragma once
#include <map>
#include <smacc2/common.hpp>
#include <utility>
#include <vector>

namespace smacc2
{
class ISmaccOrthogonal
{
public:
  void setStateMachine(ISmaccStateMachine * value);

  inline ISmaccStateMachine * getStateMachine();

  void initState(ISmaccState * state);

  void addClientBehavior(std::shared_ptr<smacc2::ISmaccClientBehavior> clientBehavior);

  // The owning state is passed explicitly so that, when several states are active at
  // once (parallel orthogonal regions), each state's client behaviors are looked up by
  // owning-state key instead of a single LIFO stack top. For a single active leaf this
  // reduces to the classic behavior.
  void runtimeConfigure(ISmaccState * state);

  void onEntry(ISmaccState * state);

  void onExit(ISmaccState * state);

  void onDispose(ISmaccState * state);

  virtual std::string getName() const;

  template <typename SmaccComponentType>
  void requiresComponent(SmaccComponentType *& storage);

  template <typename SmaccClientType>
  bool requiresClient(SmaccClientType *& storage);

  inline const std::vector<std::shared_ptr<smacc2::ISmaccClient>> & getClients();

  inline const std::map<ISmaccState *, std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>>> &
  getClientBehaviors() const;

  // Returns the client behaviors owned by a specific active state (empty if none).
  const std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>> & getClientBehaviorsForState(
    ISmaccState * state) const;

  template <typename T>
  void setGlobalSMData(std::string name, T value);

  template <typename T>
  bool getGlobalSMData(std::string name, T & ret);

  // returns a client behavior of a given type. If the client behavior is not found, it returns nullptr
  // the index parameter is used to specify the client behavior in case there are more than one
  template <typename TClientBehavior>
  TClientBehavior * getClientBehavior(int index = 0);

  rclcpp::Node::SharedPtr getNode();
  inline rclcpp::Logger getLogger() { return getNode()->get_logger(); }

protected:
  virtual void onInitialize();

  void initializeClients();

  template <typename TOrthogonal, typename TClient>
  void assignClientToOrthogonal(TClient * client);

  std::vector<std::shared_ptr<smacc2::ISmaccClient>> clients_;

private:
  ISmaccStateMachine * stateMachine_;

  // Client behaviors keyed by their owning state. Supports several simultaneously
  // active states across parallel orthogonal regions without a shared LIFO stack.
  std::map<ISmaccState *, std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>>>
    clientBehaviors_;
  friend class ISmaccStateMachine;

  std::mutex mutex_;
};

}  // namespace smacc2
