// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for all sensor filter chain nodelets.
 */

#include <string>
#include <utility>

#include <nodelet/nodelet.h>

#include <sensor_filters/FilterChainBase.h>

namespace sensor_filters
{

template<typename T, typename Base = sensor_filters::FilterChainBase<T>>
class FilterChainNodelet : public nodelet::Nodelet, public Base
{
protected:
  std::string filterChainNamespace;

public:
  explicit FilterChainNodelet(std::string filterChainNamespace) :
      nodelet::Nodelet(), Base(), filterChainNamespace(std::move(filterChainNamespace))
  {
  }

  ~FilterChainNodelet() override = default;

protected:
  void onInit() override
  {
    this->initFilters(
        this->filterChainNamespace, this->getPrivateNodeHandle(), this->getPrivateNodeHandle(), true,
        this->getPrivateNodeHandle().param("input_queue_size", 10),
        this->getPrivateNodeHandle().param("output_queue_size", 10));
  }
};

}

#define DECLARE_SENSOR_FILTER_BASE(TYPE, BASE, CONFIG) \
namespace sensor_filters \
{ \
class TYPE ## FilterChainNodelet : public FilterChainNodelet<sensor_msgs::TYPE, BASE> \
{ \
  public: TYPE ## FilterChainNodelet() : FilterChainNodelet(CONFIG "_filter_chain") {} \
}; \
}\
PLUGINLIB_EXPORT_CLASS(sensor_filters::TYPE ## FilterChainNodelet, nodelet::Nodelet)

#define DECLARE_SENSOR_FILTER(TYPE, CONFIG) \
DECLARE_SENSOR_FILTER_BASE(TYPE, sensor_filters::FilterChainBase<sensor_msgs::TYPE>, CONFIG)
