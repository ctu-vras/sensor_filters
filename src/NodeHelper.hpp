// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief This is a shim for ROS Kilted and older which hacks in a way to get a shared_ptr on Node from a raw pointer.
 *        This is needed because shared_from_this() can't be used in node constructors.
 */

#include <rclcpp/node.hpp>

namespace sensor_filters
{

rclcpp::Node::SharedPtr get_node_shared_ptr_from_raw_ptr(rclcpp::Node* node);

} // namespace sensor_filters
