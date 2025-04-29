// Copyright (c) 2024 Open Navigation LLC
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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_action_server.hpp"
#include "opennav_docking_bt/dock_robot.hpp"

class DockRobotActionServer
  : public TestActionServer<nav2_msgs::action::DockRobot>
{
public:
  DockRobotActionServer()
  : TestActionServer("dock_robot")
  {}

protected:
  void execute(
    const typename std::shared_ptr<
      rclcpp_action::ServerGoalHandle<nav2_msgs::action::DockRobot>>
    goal_handle)
  override
  {
    auto result = std::make_shared<nav2_msgs::action::DockRobot::Result>();
    bool return_success = getReturnSuccess();
    if (return_success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
};

class DockRobotActionTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("dock_robot_action_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>(
      "node",
      node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout",
      std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration",
      std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout",
      std::chrono::milliseconds(1000));
    config_->blackboard->set<bool>("initial_pose_received", false);

    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<opennav_docking_bt::DockRobotAction>(
          name, "dock_robot", config);
      };

    factory_->registerBuilder<opennav_docking_bt::DockRobotAction>(
      "DockRobot", builder);
  }

  static void TearDownTestCase()
  {
    factory_.reset();
    action_server_.reset();
    delete config_;
    config_ = nullptr;
    node_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<DockRobotActionServer> action_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr DockRobotActionTestFixture::node_ = nullptr;
std::shared_ptr<DockRobotActionServer>
DockRobotActionTestFixture::action_server_ = nullptr;
BT::NodeConfiguration * DockRobotActionTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> DockRobotActionTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> DockRobotActionTestFixture::tree_ = nullptr;

TEST_F(DockRobotActionTestFixture, test_ports)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <DockRobot />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<bool>("use_dock_id"), true);
  EXPECT_EQ(tree_->rootNode()->getInput<float>("max_staging_time"), 1000.0);
  EXPECT_EQ(tree_->rootNode()->getInput<bool>("navigate_to_staging_pose"), true);

  xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <DockRobot use_dock_id="false" dock_id="test_dock" dock_pose="0;map;1.0;2.0;3.0;4.0;5.0;6.0;7.0"
            dock_type="dock1" max_staging_time="20.0" navigate_to_staging_pose="false"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<bool>("use_dock_id"), false);
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("dock_id"), "test_dock");
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("dock_type"), "dock1");
  EXPECT_EQ(tree_->rootNode()->getInput<float>("max_staging_time"), 20.0);
  EXPECT_EQ(tree_->rootNode()->getInput<bool>("navigate_to_staging_pose"), false);

  std::vector<geometry_msgs::msg::PoseStamped> values;
  tree_->rootNode()->getInput("dock_pose", values);
  EXPECT_EQ(rclcpp::Time(values[0].header.stamp).nanoseconds(), 0);
  EXPECT_EQ(values[0].header.frame_id, "map");
  EXPECT_EQ(values[0].pose.position.x, 1.0);
  EXPECT_EQ(values[0].pose.position.y, 2.0);
  EXPECT_EQ(values[0].pose.position.z, 3.0);
  EXPECT_EQ(values[0].pose.orientation.x, 4.0);
  EXPECT_EQ(values[0].pose.orientation.y, 5.0);
  EXPECT_EQ(values[0].pose.orientation.z, 6.0);
  EXPECT_EQ(values[0].pose.orientation.w, 7.0);
}

TEST_F(DockRobotActionTestFixture, test_tick)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <DockRobot use_dock_id="true" dock_id="dock1" success="{success}"
              num_retries="{num_retries}" error_code_id="{error_code_id}" error_msg="{error_msg}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(config_->blackboard->get<bool>("success"), true);
  EXPECT_EQ(config_->blackboard->get<int>("num_retries"), 0);
  EXPECT_EQ(config_->blackboard->get<int>("error_code_id"), 0);
  EXPECT_EQ(config_->blackboard->get<std::string>("error_msg"), "");

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);
}

TEST_F(DockRobotActionTestFixture, test_tick2)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <DockRobot use_dock_id="false" dock_type="dock1"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // tick until node succeeds
  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS) {
    tree_->rootNode()->executeTick();
  }

  // the goal should have reached our server
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::SUCCESS);

  // halt node so another goal can be sent
  tree_->haltTree();
  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::IDLE);
}

TEST_F(DockRobotActionTestFixture, test_failure)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <DockRobot use_dock_id="false" dock_type="dock1"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  action_server_->setReturnSuccess(false);

  while (tree_->rootNode()->status() != BT::NodeStatus::SUCCESS &&
    tree_->rootNode()->status() != BT::NodeStatus::FAILURE)
  {
    tree_->rootNode()->executeTick();
  }

  EXPECT_EQ(tree_->rootNode()->status(), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize action server and spin on new thread
  DockRobotActionTestFixture::action_server_ =
    std::make_shared<DockRobotActionServer>();

  std::thread server_thread([]() {
      rclcpp::spin(DockRobotActionTestFixture::action_server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  return all_successful;
}
