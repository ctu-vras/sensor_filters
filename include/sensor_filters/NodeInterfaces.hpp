// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Node interfaces required by sensor filter chains.
 */

#include <rclcpp/node_interfaces/node_interfaces.hpp>

namespace sensor_filters {

using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
  rclcpp::node_interfaces::NodeBaseInterface,
  rclcpp::node_interfaces::NodeParametersInterface,
  rclcpp::node_interfaces::NodeLoggingInterface,
  rclcpp::node_interfaces::NodeTimersInterface,
  rclcpp::node_interfaces::NodeTopicsInterface
>;

} // namespace sensor_filters
