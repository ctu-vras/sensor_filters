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
class FilterChainNode : public Base
{
public:
  explicit FilterChainNode(const std::string& filterChainNamespace, ros::NodeHandle filterNodeHandle,
    ros::NodeHandle topicNodeHandle) : Base()
  {
    this->initFilters(
      filterChainNamespace, filterNodeHandle, topicNodeHandle, false,
      filterNodeHandle.param("input_queue_size", 10),
      filterNodeHandle.param("output_queue_size", 10));
  }
};


template <typename T, typename Base = sensor_filters::FilterChainBase<T>>
void spinFilterChain(const std::string& filterChainNamespace, int argc, char** argv)
{
  ros::init(argc, argv, "filter_chain");
  ros::NodeHandle nh("~");
  const FilterChainNode<T, Base> node(filterChainNamespace, nh, nh);
  ros::spin();
}

}
