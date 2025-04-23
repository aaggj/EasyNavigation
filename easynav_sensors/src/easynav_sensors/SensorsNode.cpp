// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the SensorsNode class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_sensors/SensorsNode.hpp"

namespace easynav_sensors
{

using namespace std::chrono_literals;

SensorsNode::SensorsNode(const rclcpp::NodeOptions & options)
: LifecycleNode("sensors_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_, *this, false)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
SensorsNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Configuring node...");

  declare_parameter("forget_time", 0.5);
  get_parameter("forget_time", forget_time_);

  declare_parameter("sensors", std::vector<std::string>{});
  std::vector<std::string> sensor_names;
  get_parameter("sensors", sensor_names);

  for (const auto &sensor : sensor_names) {
    std::string topic, type;
    declare_parameter(sensor + ".topic", "");
    declare_parameter(sensor + ".type", "");
    get_parameter(sensor + ".topic", topic);
    get_parameter(sensor + ".type", type);

    if (type == "LaserScan") {
      auto sub = create_subscription<sensor_msgs::msg::LaserScan>(
        topic, 10,
        [this, sensor](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          handle_laserscan(sensor, msg);
        });
      laser_subs_.push_back(sub);
    } else if (type == "PointCloud") {
      auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, 10,
        [this, sensor](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          handle_pointcloud(sensor, msg);
        });
      pc_subs_.push_back(sub);
    }
  }

  fused_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/easynav/fused_cloud", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/easynav/markers", 10);
  sensors_main_timer_ = create_wall_timer(
    100ms, std::bind(&SensorsNode::sensors_cycle, this), realtime_cbg_);
  sensors_main_timer_->cancel();
  RCLCPP_INFO(get_logger(), "Node configured successfully.");
  
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  sensors_main_timer_ = create_timer(1ms, std::bind(&SensorsNode::sensors_cycle, this),
    realtime_cbg_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  sensors_main_timer_->cancel();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
SensorsNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

void
SensorsNode::sensors_cycle()
{
}


}  // namespace easynav_sensors
