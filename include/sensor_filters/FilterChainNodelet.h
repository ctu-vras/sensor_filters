#pragma once
/*
 * Copyright (c) 2021, Czech Technical University in Prague
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <utility>

#include <nodelet/nodelet.h>

#include <sensor_filters/FilterChainBase.h>

namespace sensor_filters
{

template <typename T, typename Base = sensor_filters::FilterChainBase<T>>
class FilterChainNodelet : public nodelet::Nodelet, public Base
{
  protected: std::string filterChainNamespace;

  public: explicit FilterChainNodelet(std::string filterChainNamespace) :
    nodelet::Nodelet(),
    Base(),
    filterChainNamespace(std::move(filterChainNamespace))
  {
  }

  ~FilterChainNodelet() override = default;

  protected: void onInit() override
  {
    this->initFilters(
      this->filterChainNamespace, this->getPrivateNodeHandle(), this->getPrivateNodeHandle(), true,
      this->getPrivateNodeHandle().param("input_queue_size", 10),
      this->getPrivateNodeHandle().param("output_queue_size", 10)
    );
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
