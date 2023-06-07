// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Specialized base for PointCloud2 filter chains that uses point_cloud_transport.
 */

#include <memory>
#include <string>

#include <point_cloud_transport/point_cloud_transport.h>
#include <point_cloud_transport/publisher.h>
#include <point_cloud_transport/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_filters/FilterChainBase.h>

namespace sensor_filters
{
class PointCloud2FilterChainBase : public FilterChainBase<sensor_msgs::PointCloud2>
{
public:
  PointCloud2FilterChainBase() : FilterChainBase<sensor_msgs::PointCloud2>()
  {
  }

protected:
  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct;
  point_cloud_transport::Publisher pctPublisher;
  point_cloud_transport::Subscriber pctSubscriber;

  void initFilters(const std::string& filterChainNamespace, ros::NodeHandle filterNodeHandle,
                   ros::NodeHandle topicNodeHandle, bool useSharedPtrMessages, size_t inputQueueSize,
                   size_t outputQueueSize) override;

  void advertise() override;

  void subscribe() override;

  void publishShared(const sensor_msgs::PointCloud2ConstPtr& msg) override;
};

}
