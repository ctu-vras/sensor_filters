// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for a sensor filter chain node.
 */

#include <string>

#include <sensor_filters/FilterChainBase.h>

namespace sensor_filters
{

template <typename T, typename Base = sensor_filters::FilterChainBase<T>>
class FilterChainNode : public rclcpp::Node, public Base {
public:
  explicit FilterChainNode() : Base(), Node("") {
  }
};


template <typename T, typename Base = sensor_filters::FilterChainBase<T>>
void spinFilterChain(const std::string& filterChainNamespace, int argc, char** argv) {
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<FilterChainNode<T, Base>>();
  node->initFilters(
    filterChainNamespace, node, false,
    node->declare_parameter("input_queue_size", 10),
    node->declare_parameter("output_queue_size", 10));
  rclcpp::spin(node);
}

}
