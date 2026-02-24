// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>
#include <image_transport/image_transport.hpp>
#include <sensor_filters/FilterChainBase.hpp>
#include <sensor_filters/ImageFilterChainBase.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace sensor_filters {
    void ImageFilterChainBase::initFilters(
        const std::string& filterChainNamespace,
        rclcpp::Node::SharedPtr node,
        const bool useSharedPtrMessages,
        const long inputQueueSize,
        const long outputQueueSize
    ) {
        // TODO 2026-02-23 (solonovamax): offer parameter to advertise camera topic?
        this->it = std::make_unique<image_transport::ImageTransport>(node);
        FilterChainBase::initFilters(filterChainNamespace, node, useSharedPtrMessages, inputQueueSize, outputQueueSize);
    }

    void ImageFilterChainBase::advertise() {
        this->itPublisher = this->it->advertise("output", this->outputQueueSize);
    }

    void ImageFilterChainBase::subscribe() {
        this->itSubscriber = this->it->subscribe(
            "input", this->inputQueueSize,
            [this](const typename sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                this->callbackShared(msg);
            }
        );
    }

    void ImageFilterChainBase::publishShared(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        this->itPublisher.publish(msg);
    }
}
