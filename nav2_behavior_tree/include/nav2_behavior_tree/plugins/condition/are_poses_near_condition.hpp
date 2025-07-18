// Copyright (c) 2025 Open Navigation LLC
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_POSES_NEAR_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_POSES_NEAR_CONDITION_HPP_

#include <string>
#include <memory>

#include "nav2_ros_common/lifecycle_node.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "tf2_ros/buffer.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class ArePosesNearCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ArePosesNearCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ArePosesNearCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief A destructor for nav2_behavior_tree::ArePosesNearCondition
   */
  ~ArePosesNearCondition() override = default;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Checks if the current robot pose lies within a given distance from the goal
   * @return bool true when goal is reached, false otherwise
   */
  bool arePosesNearby();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("ref_pose", "Destination"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Destination"),
      BT::InputPort<std::string>("global_frame", "Global frame"),
      BT::InputPort<double>("tolerance", 0.5, "Tolerance")
    };
  }

private:
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
  std::string global_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__ARE_POSES_NEAR_CONDITION_HPP_
