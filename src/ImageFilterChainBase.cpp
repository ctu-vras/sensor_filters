// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <image_transport/image_transport.hpp>
#include <sensor_filters/FilterChainBase.hpp>
#include <sensor_filters/ImageFilterChainBase.hpp>
#include <sensor_filters/NodeInterfaces.hpp>
#include <sensor_msgs/msg/image.hpp>

#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
#include "NodeHelper.hpp"
#endif

namespace sensor_filters {

    ImageFilterChainBase::ImageFilterChainBase(RequiredInterfaces nodeInterfaces,
        const std::string& name,
        const FilterChainOptions& defaultChainOptions): FilterChainBase(nodeInterfaces, name, defaultChainOptions)
    {
        const auto params = nodeInterfaces.get_node_parameters_interface();
        // image_transport does not declare the parameter
        params->declare_parameter("image_transport", rclcpp::ParameterValue("raw"));
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
        this->nodePtr = get_node_shared_ptr_from_interfaces(nodeInterfaces);
        this->it = std::make_unique<image_transport::ImageTransport>(this->nodePtr);
#else
        this->it = std::make_unique<image_transport::ImageTransport>(nodeInterfaces);
#endif
    }

    bool ImageFilterChainBase::validateSubscriptionType() const
    {
        return this->options.subscriptionType == MessagePassingType::SHARED_PTR;
    }

    bool ImageFilterChainBase::validatePublicationType() const
    {
        return true;
    }

    void ImageFilterChainBase::advertise(const std::string& topic)
    {
        const auto topics = this->nodeInterfaces.get_node_topics_interface();

#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
        this->itPublisher = image_transport::create_publisher(this->nodePtr.get(), topics->resolve_topic_name(topic),
            rclcpp::QoS(this->options.outputQueueSize).get_rmw_qos_profile()
#ifndef IMAGE_TRANSPORT_PUB_OPTIONS_NOT_AVAILABLE
            , this->publisherOptions
#endif
            );
#else
        this->itPublisher = image_transport::create_publisher(this->nodeInterfaces, topics->resolve_topic_name(topic),
            rclcpp::QoS(this->options.outputQueueSize), this->publisherOptions);
#endif
    }

    void ImageFilterChainBase::unadvertise()
    {
        this->itPublisher.shutdown();
    }

    void ImageFilterChainBase::subscribe(const std::string& topic)
    {
        const auto topics = this->nodeInterfaces.get_node_topics_interface();

        this->itSubscriber = this->it->subscribe(
            topics->resolve_topic_name(topic), this->options.inputQueueSize,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                this->callbackShared(msg);
            }, {}, nullptr, this->subscriptionOptions);
    }

    void ImageFilterChainBase::unsubscribe()
    {
        this->itSubscriber.shutdown();
    }

    void ImageFilterChainBase::publishUnique(sensor_msgs::msg::Image::UniquePtr msg)
    {
        this->itPublisher.publish(std::move(msg));
    }

    void ImageFilterChainBase::publishShared(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        this->itPublisher.publish(msg);
    }

    void ImageFilterChainBase::publishReference(const sensor_msgs::msg::Image& msg)
    {
        this->itPublisher.publish(msg);
    }

}  // namespace sensor_filters
