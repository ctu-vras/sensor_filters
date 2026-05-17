// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>

#include <sensor_filters/FilterChainNode.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
#include "NodeHelper.hpp"
#endif

namespace sensor_filters {
    class PointCloud2FilterChainNode : public FilterChainNode<sensor_msgs::msg::PointCloud2> {
    public:
        explicit PointCloud2FilterChainNode(
            const rclcpp::NodeOptions& options,
            const FilterChainOptions& defaultChainOptions = {
                10U, 10U, MessagePassingType::SHARED_PTR, MessagePassingType::SHARED_PTR
            })
            : FilterChainNode("sensor_msgs::msg::PointCloud2", "pointcloud2_filter_chain", options, defaultChainOptions) {
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
            this->nodePtr = get_node_shared_ptr_from_raw_ptr(this);
            this->pct = std::make_unique<point_cloud_transport::PointCloudTransport>(this->nodePtr);
#else
            this->pct = std::make_unique<point_cloud_transport::PointCloudTransport>(*this);
#endif
            PointCloud2FilterChainNode::configure();
        }

    protected:
        bool validateSubscriptionType() const override
        {
            return this->options.subscriptionType == MessagePassingType::SHARED_PTR;
        }

        bool validatePublicationType() const override
        {
            return this->options.publicationType != MessagePassingType::UNIQUE_PTR;
        }

        void advertise() override {
            this->pctPublisher = this->pct->advertise(
                this->get_node_topics_interface()->resolve_topic_name("output"), this->options.outputQueueSize);
        }

        void subscribe() override {
            this->pctSubscriber = this->pct->subscribe(
                this->get_node_topics_interface()->resolve_topic_name("input"), this->options.inputQueueSize,
                [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
                    PointCloud2FilterChainNode::callbackShared(msg);
                }
            );
        }

        void publishUnique(sensor_msgs::msg::PointCloud2::UniquePtr) override {
            throw std::runtime_error("PointCloud2FilterChainNode does not support unique_ptr publications");
        }

        void publishShared(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) override {
            this->pctPublisher.publish(msg);
        }

        void publishReference(const sensor_msgs::msg::PointCloud2& msg) override {
            this->pctPublisher.publish(msg);
        }

    private:
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
	    rclcpp::Node::SharedPtr nodePtr;
#endif
        std::unique_ptr<point_cloud_transport::PointCloudTransport> pct;
        point_cloud_transport::Publisher pctPublisher;
        point_cloud_transport::Subscriber pctSubscriber;
    };
} // namespace sensor_filters

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_filters::PointCloud2FilterChainNode)
