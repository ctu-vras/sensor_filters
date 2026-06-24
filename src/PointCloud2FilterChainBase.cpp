// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_filters/FilterChainBase.hpp>
#include <sensor_filters/NodeInterfaces.hpp>
#include <sensor_filters/PointCloud2FilterChainBase.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
#include "NodeHelper.hpp"
#endif

namespace sensor_filters {

    PointCloud2FilterChainBase::PointCloud2FilterChainBase(RequiredInterfaces nodeInterfaces,
        const std::string& name,
        const FilterChainOptions& defaultChainOptions): FilterChainBase(nodeInterfaces, name, defaultChainOptions)
    {
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
        this->nodePtr = get_node_shared_ptr_from_interfaces(nodeInterfaces);
        this->pct = std::make_unique<point_cloud_transport::PointCloudTransport>(this->nodePtr);
        this->transportHints = std::make_unique<point_cloud_transport::TransportHints>(this->nodePtr);
#else
        this->pct = std::make_unique<point_cloud_transport::PointCloudTransport>(nodeInterfaces);
        this->transportHints = std::make_unique<point_cloud_transport::TransportHints>(this->nodeInterfaces);
#endif
    }

    bool PointCloud2FilterChainBase::validateSubscriptionType() const
    {
        return this->options.subscriptionType == MessagePassingType::SHARED_PTR;
    }

    bool PointCloud2FilterChainBase::validatePublicationType() const
    {
        return this->options.publicationType != MessagePassingType::UNIQUE_PTR;
    }

    void PointCloud2FilterChainBase::advertise(const std::string& topic)
    {
        const auto topics = this->nodeInterfaces.get_node_topics_interface();

        this->pctPublisher = this->pct->advertise(
            topics->resolve_topic_name(topic), this->options.outputQueueSize, this->publisherOptions);
    }

    void PointCloud2FilterChainBase::unadvertise()
    {
        this->pctPublisher.shutdown();
    }

    void PointCloud2FilterChainBase::subscribe(const std::string& topic)
    {
        const auto topics = this->nodeInterfaces.get_node_topics_interface();

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
        this->pctSubscriber = point_cloud_transport::create_subscription(
            this->nodePtr, topics->resolve_topic_name(topic),
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
                this->callbackShared(msg);
            },
            this->pct->getTransportOrDefault(this->transportHints.get()),
            rclcpp::QoS(this->options.inputQueueSize).get_rmw_qos_profile(), this->subscriptionOptions);
#else
        this->pctSubscriber = this->pct->subscribe(
            topics->resolve_topic_name(topic), this->options.inputQueueSize,
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
                this->callbackShared(msg);
            }, {}, nullptr, this->subscriptionOptions);
#endif
    }

    void PointCloud2FilterChainBase::unsubscribe()
    {
        this->pctSubscriber.shutdown();
    }

    bool PointCloud2FilterChainBase::isSubscribed() const
    {
        return this->pctSubscriber;
    }

    size_t PointCloud2FilterChainBase::getNumSubscribers() const
    {
        if (!this->pctPublisher)
            return 0u;

        size_t count {0u};
        for (const auto& [topic, pub] : this->pctPublisher.getPublishers())
            count += pub->get_subscription_count() + pub->get_subscription_count();
        return count;
    }

    void PointCloud2FilterChainBase::publishUnique(sensor_msgs::msg::PointCloud2::UniquePtr)
    {
        throw std::runtime_error("PointCloud2FilterChainNode does not support unique_ptr publications");
    }

    void PointCloud2FilterChainBase::publishShared(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
        this->pctPublisher.publish(msg);
    }

    void PointCloud2FilterChainBase::publishReference(const sensor_msgs::msg::PointCloud2& msg)
    {
        this->pctPublisher.publish(msg);
    }

} // namespace sensor_filters
