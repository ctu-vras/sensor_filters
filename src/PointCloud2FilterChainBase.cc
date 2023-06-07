// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <point_cloud_transport/point_cloud_transport.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>

#include <sensor_filters/FilterChainBase.h>
#include <sensor_filters/PointCloud2FilterChainBase.h>

namespace sensor_filters
{

void PointCloud2FilterChainBase::initFilters(const std::string& filterChainNamespace, ros::NodeHandle filterNodeHandle,
                                             ros::NodeHandle topicNodeHandle, const bool useSharedPtrMessages,
                                             const size_t inputQueueSize, const size_t outputQueueSize)
{
  this->pct = std::make_unique<point_cloud_transport::PointCloudTransport>(topicNodeHandle);
  FilterChainBase::initFilters(filterChainNamespace, filterNodeHandle, topicNodeHandle, useSharedPtrMessages,
                               inputQueueSize, outputQueueSize);
}

void PointCloud2FilterChainBase::advertise()
{
  const auto resolvedOutput = this->topicNodeHandle.resolveName("output");
  this->pctPublisher = this->pct->advertise(resolvedOutput, this->outputQueueSize);
}

void PointCloud2FilterChainBase::subscribe()
{
  const auto resolvedInput = this->topicNodeHandle.resolveName("input");
  this->pctSubscriber = this->pct->subscribe<PointCloud2FilterChainBase>(
      resolvedInput, this->inputQueueSize, &PointCloud2FilterChainBase::callbackShared, this);
}

void PointCloud2FilterChainBase::publishShared(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  this->pctPublisher.publish(msg);
}

}
