// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATED_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATED_CONTROLLER_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child if the goal was updated
 * @note This is an Asynchronous (long-running) node which may return a RUNNING state while executing.
 *       It will re-initialize when halted.
 */
class GoalUpdatedController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::GoalUpdatedController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GoalUpdatedController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    // Register JSON definitions for the types used in the ports
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();

    return {
      BT::InputPort<nav_msgs::msg::Goals>(
        "goals", "Vector of navigation goals"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Navigation goal"),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  bool goal_was_updated_;
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Goals goals_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATED_CONTROLLER_HPP_
