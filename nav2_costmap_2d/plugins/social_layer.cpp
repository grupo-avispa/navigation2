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

#include "nav2_costmap_2d/social_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::SocialLayer, nav2_costmap_2d::Layer)

using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

SocialLayer::SocialLayer()
{
}

SocialLayer::~SocialLayer()
{
  auto node = node_.lock();
  if (dyn_params_handler_ && node) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

void SocialLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("people_topic", rclcpp::ParameterValue("people"));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "people_topic", people_topic_);

  people_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    people_topic_, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
      people_poses_ = *msg;
    });

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SocialLayer::dynamicParametersCallback, this, std::placeholders::_1));

  current_ = true;
}

void SocialLayer::reset()
{
  current_ = false;
}

void SocialLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * /*min_x*/,
  double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!enabled_) {
    return;
  }
}

void SocialLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  current_ = true;
}

void SocialLayer::matchSize()
{

}

bool SocialLayer::isClearable()
{
  return false;
}

void SocialLayer::onFootprintChanged()
{

}

rcl_interfaces::msg::SetParametersResult
SocialLayer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();

    if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        current_ = false;
      }
    }
  }
  result.successful = true;
  return result;
}

double SocialLayer::gaussian(
  double x, double y, double x0, double y0, double A, double varx, double vary, double skew)
{
  double dx = x - x0, dy = y - y0;
  double h = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double mx = cos(angle - skew) * h;
  double my = sin(angle - skew) * h;
  double f1 = pow(mx, 2.0) / (2.0 * varx);
  double f2 = pow(my, 2.0) / (2.0 * vary);
  return A * exp(-(f1 + f2));
}

} // namespace nav2_costmap_2d
