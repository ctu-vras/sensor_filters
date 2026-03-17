// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <string>
#include <sensor_filters/FilterChainNode.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sensor_filters {
    class PointCloud2FilterChainNode : public FilterChainNode<sensor_msgs::msg::PointCloud2> {
    public:
        explicit PointCloud2FilterChainNode(const rclcpp::NodeOptions& options) : FilterChainNode("pointcloud2_filter_chain", options) {
            this->pct = std::make_unique<point_cloud_transport::PointCloudTransport>(this->shared_from_this());
            PointCloud2FilterChainNode::configure();
        }

        [[deprecated]] explicit PointCloud2FilterChainNode() : FilterChainNode("pointcloud2_filter_chain") {
            this->pct = std::make_unique<point_cloud_transport::PointCloudTransport>(this->shared_from_this());
            PointCloud2FilterChainNode::configure();
        }

    protected:
        void advertise() override {
            this->pctPublisher = this->pct->advertise("output", this->outputQueueSize);
        }

        void subscribe() override {
            this->pctSubscriber = this->pct->subscribe(
                "input", this->inputQueueSize,
                [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
                    PointCloud2FilterChainNode::callbackShared(msg);
                }
            );
        }

        void publishShared(const typename sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) override {
            this->pctPublisher.publish(msg);
        }

    private:
        std::unique_ptr<point_cloud_transport::PointCloudTransport> pct;
        point_cloud_transport::Publisher pctPublisher;
        point_cloud_transport::Subscriber pctSubscriber;
    };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_filters::PointCloud2FilterChainNode)
