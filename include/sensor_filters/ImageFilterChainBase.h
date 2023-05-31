// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Specialized base for Image filter chains that uses image_transport.
 */

#include <memory>

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/Image.h>

#include <sensor_filters/FilterChainBase.h>

namespace sensor_filters
{
class ImageFilterChainBase : public FilterChainBase<sensor_msgs::Image>
{
  public: ImageFilterChainBase() : FilterChainBase<sensor_msgs::Image>() {}

protected:
    std::unique_ptr<image_transport::ImageTransport> it;
    image_transport::Publisher itPublisher;
    image_transport::Subscriber itSubscriber;

    void initFilters(const std::string &filterChainNamespace, ros::NodeHandle filterNodeHandle,
                     ros::NodeHandle topicNodeHandle, const bool useSharedPtrMessages, const size_t inputQueueSize,
                     const size_t outputQueueSize) override;

    void advertise() override;

    void subscribe() override;

    void publishShared(const sensor_msgs::ImageConstPtr& msg) override;
};

}
