// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <sensor_filters/FilterChainBase.h>
#include <sensor_filters/PointCloud2FilterChainBase.h>

namespace sensor_filters
{

void PointCloud2FilterChainBase::initFilters(const std::string& filterChainNamespace, rclcpp::Node::SharedPtr node,
                                             const bool useSharedPtrMessages, const long inputQueueSize, const long outputQueueSize)
{
  this->pct = std::make_unique<point_cloud_transport::PointCloudTransport>(node);
  FilterChainBase::initFilters(filterChainNamespace, node, useSharedPtrMessages,
                               inputQueueSize, outputQueueSize);
}

void PointCloud2FilterChainBase::advertise()
{
  this->pctPublisher = this->pct->advertise("output", this->outputQueueSize);
}

void PointCloud2FilterChainBase::subscribe()
{
  this->pctSubscriber = this->pct->subscribe(
    "input", this->inputQueueSize, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    PointCloud2FilterChainBase::callbackShared(msg);
  });
}

void PointCloud2FilterChainBase::publishShared(const typename sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  this->pctPublisher.publish(msg);
}
}
