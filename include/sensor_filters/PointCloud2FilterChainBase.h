// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Specialized base for PointCloud2 filter chains that uses point_cloud_transport.
 */

#include <memory>
#include <string>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/subscriber.hpp>
#include <sensor_filters/FilterChainBase.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sensor_filters {
    class PointCloud2FilterChainBase : public FilterChainBase<sensor_msgs::msg::PointCloud2> {
    public:
        PointCloud2FilterChainBase() {}

        void initFilters(
            const std::string& filterChainNamespace,
            rclcpp::Node::SharedPtr node,
            bool useSharedPtrMessages,
            long inputQueueSize,
            long outputQueueSize
        ) override;

    protected:
        std::unique_ptr<point_cloud_transport::PointCloudTransport> pct;
        point_cloud_transport::Publisher pctPublisher;
        point_cloud_transport::Subscriber pctSubscriber;

        void advertise() override;

        void subscribe() override;

        void publishShared(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) override;
    };
}
