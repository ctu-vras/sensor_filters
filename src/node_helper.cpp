// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief This is a shim for ROS Kilted and older which hacks in a way to get a shared_ptr on Node from a raw pointer.
 *        This is needed because shared_from_this() can't be used in node constructors.
 */

#include <sstream>
#include <string>

#define private public
#include <rclcpp/node.hpp>
#undef private

namespace sensor_filters
{

rclcpp::Node::SharedPtr get_node_shared_ptr_from_raw_ptr(rclcpp::Node* node)
{
  auto shared_node = node->create_sub_node("sub");
  const_cast<std::string&>(shared_node->effective_namespace_) = node->get_effective_namespace();
  const_cast<std::string&>(shared_node->sub_namespace_) = node->get_sub_namespace();
  return shared_node;
}

} // namespace sensor_filters
