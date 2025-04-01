// Copyright (c) 2025 Alberto J. Tudela Roldán
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


#ifndef NAV2_COSTMAP_2D__SOCIAL_LAYER_HPP_
#define NAV2_COSTMAP_2D__SOCIAL_LAYER_HPP_

#include <mutex>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

namespace nav2_costmap_2d
{

/**
 * @class SocialLayer
 * @brief A layer that can be used to add proxemic information to the costmap
 *        (e.g. social distancing, etc.)
 */
class SocialLayer : public CostmapLayer
{
public:
  /**
   * @brief Social Layer constructor
   */
  SocialLayer();

  /**
   * @brief Social Layer destructor
   */
  ~SocialLayer();

    /**
   * @brief Initialization process of layer on startup
   */
  void onInitialize() override;

  /**
   * @brief Reset this costmap
   */
  void reset() override;

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Match the size of the master costmap
   */
  void matchSize() override;

  /**
   * @brief Reports that no clearing operation is required
   */
  bool isClearable() override;

protected:
  /**
   * @brief Process updates on footprint changes to the inflation layer
   */
  void onFootprintChanged() override;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  /**
   * @brief Compute the Gaussian function
   *
   * @param x X coordinate
   * @param y Y coordinate
   * @param x0 X center of the Gaussian
   * @param y0 Y center of the Gaussian
   * @param A Amplitude of the Gaussian
   * @param varx Variance in the X direction
   * @param vary Variance in the Y direction
   * @param skew Skew factor
   * @return double The computed Gaussian value
   */
  double gaussian(
    double x, double y, double x0, double y0, double A, double varx, double vary, double skew);

  // The topic to subscribe to for the people poses
  std::string people_topic_;
    // Subscription to the people poses topic
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr people_sub_;
  // The list of people poses that will be used to compute the proxemics
  geometry_msgs::msg::PoseArray people_poses_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

} // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__SOCIAL_LAYER_HPP_
