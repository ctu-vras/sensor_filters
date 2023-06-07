// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <image_transport/image_transport.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>

#include <sensor_filters/FilterChainBase.h>
#include <sensor_filters/ImageFilterChainBase.h>

namespace sensor_filters
{

void ImageFilterChainBase::initFilters(const std::string& filterChainNamespace, ros::NodeHandle filterNodeHandle,
                                       ros::NodeHandle topicNodeHandle, const bool useSharedPtrMessages,
                                       const size_t inputQueueSize, const size_t outputQueueSize)
{
  this->it = std::make_unique<image_transport::ImageTransport>(topicNodeHandle);
  FilterChainBase::initFilters(filterChainNamespace, filterNodeHandle, topicNodeHandle, useSharedPtrMessages,
                               inputQueueSize, outputQueueSize);
}

void ImageFilterChainBase::advertise()
{
  const auto resolvedOutput = this->topicNodeHandle.resolveName("output");
  this->itPublisher = this->it->advertise(resolvedOutput, this->outputQueueSize);
}

void ImageFilterChainBase::subscribe()
{
  const auto resolvedInput = this->topicNodeHandle.resolveName("input");
  this->itSubscriber = this->it->subscribe<ImageFilterChainBase>(
      resolvedInput, this->inputQueueSize, &ImageFilterChainBase::callbackShared, this);
}

void ImageFilterChainBase::publishShared(const sensor_msgs::ImageConstPtr& msg)
{
  this->itPublisher.publish(msg);
}

}
