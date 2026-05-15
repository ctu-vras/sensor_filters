// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>
#include <sensor_filters/FilterChainNode.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "node_helper.hpp"

namespace sensor_filters {
    class ImageFilterChainNode : public FilterChainNode<sensor_msgs::msg::Image> {
    public:
        explicit ImageFilterChainNode(const rclcpp::NodeOptions& options) : FilterChainNode("sensor_msgs::msg::Image", "image_filter_chain", options) {
            this->nodePtr = get_node_shared_ptr_from_raw_ptr(this);
            // image_transport does not declare the parameter
            this->declare_parameter("image_transport", "raw");
            this->it = std::make_unique<image_transport::ImageTransport>(this->nodePtr);
            ImageFilterChainNode::configure();
        }

        [[deprecated]] explicit ImageFilterChainNode() : FilterChainNode("sensor_msgs::msg::Image", "image_filter_chain") {
            this->nodePtr = get_node_shared_ptr_from_raw_ptr(this);
            // image_transport does not declare the parameter
            this->declare_parameter("image_transport", "raw");
            this->it = std::make_unique<image_transport::ImageTransport>(this->nodePtr);
            ImageFilterChainNode::configure();
        }

    protected:
        void advertise() override {
            this->itPublisher = this->it->advertise(
                this->get_node_topics_interface()->resolve_topic_name("output"), this->outputQueueSize);
        }

        void subscribe() override {
            this->itSubscriber = this->it->subscribe(
                this->get_node_topics_interface()->resolve_topic_name("input"), this->inputQueueSize,
                [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                    this->callbackShared(msg);
                }
            );
        }

        void publishShared(const sensor_msgs::msg::Image::ConstSharedPtr& msg) override {
            this->itPublisher.publish(msg);
        }

    private:
        rclcpp::Node::SharedPtr nodePtr;
        std::unique_ptr<image_transport::ImageTransport> it;
        image_transport::Publisher itPublisher;
        image_transport::Subscriber itSubscriber;
    };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_filters::ImageFilterChainNode)
