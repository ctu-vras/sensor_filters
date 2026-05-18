// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <image_transport/image_transport.hpp>
#include <sensor_filters/FilterChainNode.hpp>
#include <sensor_filters/NodeInterfaces.hpp>
#include <sensor_msgs/msg/image.hpp>

#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
#include "NodeHelper.hpp"
#endif

namespace sensor_filters {
    class ImageFilterChainBase : public FilterChainBase<sensor_msgs::msg::Image> {
    public:
        explicit ImageFilterChainBase(
            RequiredInterfaces nodeInterfaces,
            const std::string& messageType = "sensor_msgs::msg::Image",
            const std::string& name = "image_filter_chain",
            const FilterChainOptions& defaultChainOptions = {
                10U, 10U, MessagePassingType::SHARED_PTR, MessagePassingType::SHARED_PTR
            })
            : FilterChainBase(nodeInterfaces, messageType, name, defaultChainOptions)
        {
            const auto params = nodeInterfaces.get_node_parameters_interface();
            // image_transport does not declare the parameter
            params->declare_parameter("image_transport", rclcpp::ParameterValue("raw"));
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
            this->nodePtr = get_node_shared_ptr_from_interfaces(nodeInterfaces);
            this->it = std::make_unique<image_transport::ImageTransport>(this->nodePtr);
#else
            this->it = std::make_unique<image_transport::ImageTransport>(*this);
#endif
        }

    protected:
        bool validateSubscriptionType() const override
        {
            return this->options.subscriptionType == MessagePassingType::SHARED_PTR;
        }

        bool validatePublicationType() const override
        {
            return true;
        }

        void advertise(const std::string& topic) override {
            const auto topics = this->nodeInterfaces.get_node_topics_interface();
            this->itPublisher = this->it->advertise(topics->resolve_topic_name(topic), this->options.outputQueueSize);
        }

        void unadvertise() override {
            this->itPublisher.shutdown();
        }

        void subscribe(const std::string& topic) override {
            const auto topics = this->nodeInterfaces.get_node_topics_interface();
            this->itSubscriber = this->it->subscribe(topics->resolve_topic_name(topic), this->options.inputQueueSize,
                [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                    this->callbackShared(msg);
                }
            );
        }

        void unsubscribe() override {
            this->itSubscriber.shutdown();
        }

        void publishUnique(sensor_msgs::msg::Image::UniquePtr msg) override {
            this->itPublisher.publish(std::move(msg));
        }

        void publishShared(const sensor_msgs::msg::Image::ConstSharedPtr& msg) override {
            this->itPublisher.publish(msg);
        }

        void publishReference(const sensor_msgs::msg::Image& msg) override {
            this->itPublisher.publish(msg);
        }

    private:
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
        rclcpp::Node::SharedPtr nodePtr;
#endif
        std::unique_ptr<image_transport::ImageTransport> it;
        image_transport::Publisher itPublisher;
        image_transport::Subscriber itSubscriber;
    };

    class ImageFilterChainNode : public FilterChainNode<sensor_msgs::msg::Image, ImageFilterChainBase> {
    public:
        explicit ImageFilterChainNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
            const FilterChainOptions& defaultChainOptions = {
                10U, 10U, MessagePassingType::SHARED_PTR, MessagePassingType::SHARED_PTR
            })
            : FilterChainNode("sensor_msgs::msg::Image", "image_filter_chain", options, defaultChainOptions)
        {
        }
    };

    class LifecycleImageFilterChainNode : public LifecycleFilterChainNode<sensor_msgs::msg::Image, ImageFilterChainBase> {
    public:
        explicit LifecycleImageFilterChainNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
            const FilterChainOptions& defaultChainOptions = {
                10U, 10U, MessagePassingType::SHARED_PTR, MessagePassingType::SHARED_PTR
            })
            : LifecycleFilterChainNode("sensor_msgs::msg::Image", "image_filter_chain", options, defaultChainOptions)
        {
        }
    };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_filters::ImageFilterChainNode)
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_filters::LifecycleImageFilterChainNode)
