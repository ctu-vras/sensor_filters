// SPDX-License-Identifier: Unlicense
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief An example how you can write a custom lifecycle node that runs the sensor filter chain and does some other
 *        tasks. This node will read its configuration from parameter `my_filter_chain`.
 */

#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_filters/FilterChainNode.hpp>

class MyLifecycleNode : public sensor_filters::LifecycleFilterChainNode<sensor_msgs::msg::LaserScan> {
public:
  explicit MyLifecycleNode(const rclcpp::NodeOptions& options) : LifecycleFilterChainNode("my_filter_chain", options) {}
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MyLifecycleNode)
