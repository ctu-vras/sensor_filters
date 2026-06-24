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
        constexpr static FilterChainOptions kDefaultChainOptions = {
            10U, 10U, MessagePassingType::SHARED_PTR, MessagePassingType::SHARED_PTR
        };

        explicit ImageFilterChainBase(RequiredInterfaces node_interfaces,
            const std::string& name = "image_filter_chain",
            const FilterChainOptions& default_chain_options = kDefaultChainOptions);

        void on_configure() override;

    protected:
        bool ValidateSubscriptionType() const override;
        bool ValidatePublicationType() const override;

        void Advertise(const std::string& topic) override;
        void Unadvertise() override;

        void Subscribe(const std::string& topic) override;
        void Unsubscribe() override;
        bool IsSubscribed() const override;
        size_t GetNumSubscribers() const override;

        void PublishUnique(sensor_msgs::msg::Image::UniquePtr msg) override;
        void PublishShared(const sensor_msgs::msg::Image::ConstSharedPtr& msg) override;
        void PublishReference(const sensor_msgs::msg::Image& msg) override;

    private:
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
        rclcpp::Node::SharedPtr node_ptr_;
#endif
        std::unique_ptr<image_transport::ImageTransport> it_;
        std::unique_ptr<image_transport::TransportHints> transport_hints_;
        image_transport::Publisher it_publisher_;
        image_transport::Subscriber it_subscriber_;
    };

}  // namespace sensor_filters
