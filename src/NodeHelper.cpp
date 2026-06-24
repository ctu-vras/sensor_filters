// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a shim for ROS Kilted and older which hacks in a way to get a shared_ptr on Node from a raw pointer.
 *        This is needed because shared_from_this() can't be used in node constructors.
 */

#include <sstream>
#define private public
#include <rclcpp/node.hpp>
#undef private

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/node_interfaces/node_interfaces.hpp>

#include "NodeHelper.hpp"

namespace sensor_filters {

struct NodeLike {
  uint8_t data[sizeof(rclcpp::Node)];
};

rclcpp::Node::SharedPtr GetNodeSharedPtrFromInterfaces(RequiredInterfaces node_interfaces) {
  // This is a trick to create a shared_ptr to Node without calling its constructor. This is super dangerous.
  // Only use it when you know what you're doing. Using any other interfaces on the node than those specified here
  // will lead to segfaults.
  const auto node_like = std::make_shared<NodeLike>();
  std::fill_n(node_like->data, sizeof(NodeLike), 0);
  auto node = std::reinterpret_pointer_cast<rclcpp::Node>(node_like);

  // This list is crafted to satisfy both image transport and point cloud transport.
  node->node_base_ = node_interfaces.get_node_base_interface();
  node->node_logging_ = node_interfaces.get_node_logging_interface();
  node->node_parameters_ = node_interfaces.get_node_parameters_interface();
  node->node_timers_ = node_interfaces.get_node_timers_interface();
  node->node_topics_ = node_interfaces.get_node_topics_interface();

  return node;
}

rclcpp::Node::SharedPtr GetNodeSharedPtrFromRawPtr(rclcpp::Node* node) {
  auto shared_node = node->create_sub_node("sub");
  const_cast<std::string&>(shared_node->effective_namespace_) = node->get_effective_namespace();
  const_cast<std::string&>(shared_node->sub_namespace_) = node->get_sub_namespace();
  return shared_node;
}

}  // namespace sensor_filters
