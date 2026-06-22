// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <memory>
#include <string>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/node.hpp>
#include <sensor_filters/FilterChainBase.hpp>
#include <sensor_filters/NodeInterfaces.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sensor_filters {

    class PointCloud2FilterChainBase : public FilterChainBase<sensor_msgs::msg::PointCloud2> {
    public:
        constexpr static FilterChainOptions DEFAULT_CHAIN_OPTIONS = {
            10U, 10U, MessagePassingType::SHARED_PTR, MessagePassingType::SHARED_PTR
        };

        explicit PointCloud2FilterChainBase(RequiredInterfaces nodeInterfaces,
            const std::string& name = "pointcloud2_filter_chain",
            const FilterChainOptions& defaultChainOptions = DEFAULT_CHAIN_OPTIONS);

    protected:
        bool validateSubscriptionType() const override;
        bool validatePublicationType() const override;

        void advertise(const std::string& topic) override;
        void unadvertise() override;

        void subscribe(const std::string& topic) override;
        void unsubscribe() override;

        void publishUnique(sensor_msgs::msg::PointCloud2::UniquePtr) override;
        void publishShared(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) override;
        void publishReference(const sensor_msgs::msg::PointCloud2& msg) override;

    private:
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
	    rclcpp::Node::SharedPtr nodePtr;
#endif
        std::unique_ptr<point_cloud_transport::PointCloudTransport> pct;
        point_cloud_transport::Publisher pctPublisher;
        point_cloud_transport::Subscriber pctSubscriber;
    };

} // namespace sensor_filters
