// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

#include <memory>
#include <string>

#include <image_transport/image_transport.hpp>
#include <sensor_filters/FilterChainNode.hpp>
#include <sensor_filters/NodeInterfaces.hpp>
#include <sensor_msgs/msg/image.hpp>


namespace sensor_filters {

    class ImageFilterChainBase : public FilterChainBase<sensor_msgs::msg::Image> {
    public:
        constexpr static FilterChainOptions DEFAULT_CHAIN_OPTIONS = {
            10U, 10U, MessagePassingType::SHARED_PTR, MessagePassingType::SHARED_PTR
        };

        explicit ImageFilterChainBase(RequiredInterfaces nodeInterfaces,
            const std::string& name = "image_filter_chain",
            const FilterChainOptions& defaultChainOptions = DEFAULT_CHAIN_OPTIONS);

        void on_configure() override;

    protected:
        bool validateSubscriptionType() const override;
        bool validatePublicationType() const override;

        void advertise(const std::string& topic) override;
        void unadvertise() override;

        void subscribe(const std::string& topic) override;
        void unsubscribe() override;
        bool isSubscribed() const override;
        size_t getNumSubscribers() const override;

        void publishUnique(sensor_msgs::msg::Image::UniquePtr msg) override;
        void publishShared(const sensor_msgs::msg::Image::ConstSharedPtr& msg) override;
        void publishReference(const sensor_msgs::msg::Image& msg) override;

    private:
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
        rclcpp::Node::SharedPtr nodePtr;
#endif
        std::unique_ptr<image_transport::ImageTransport> it;
        std::unique_ptr<image_transport::TransportHints> transportHints;
        image_transport::Publisher itPublisher;
        image_transport::Subscriber itSubscriber;
    };

}  // namespace sensor_filters
