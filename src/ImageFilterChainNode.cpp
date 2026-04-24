// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>
#include <sensor_filters/FilterChainNode.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace sensor_filters {
    class ImageFilterChainNode : public FilterChainNode<sensor_msgs::msg::Image> {
    public:
        explicit ImageFilterChainNode(const rclcpp::NodeOptions& options) : FilterChainNode("sensor_msgs::msg::Image", "Image_filter_chain", options) {
            this->it = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());
            ImageFilterChainNode::configure();
        }

        [[deprecated]] explicit ImageFilterChainNode() : FilterChainNode("sensor_msgs::msg::Image", "Image_filter_chain") {
            this->it = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());
            ImageFilterChainNode::configure();
        }

    protected:
        void advertise() override {
            this->itPublisher = this->it->advertise("output", this->outputQueueSize);
        }

        void subscribe() override {
            this->itSubscriber = this->it->subscribe(
                "input", this->inputQueueSize,
                [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
                    this->callbackShared(msg);
                }
            );
        }

        void publishShared(const sensor_msgs::msg::Image::ConstSharedPtr& msg) override {
            this->itPublisher.publish(msg);
        }

    private:
        std::unique_ptr<image_transport::ImageTransport> it;
        image_transport::Publisher itPublisher;
        image_transport::Subscriber itSubscriber;
    };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_filters::ImageFilterChainNode)
