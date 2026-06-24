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
  constexpr static FilterChainOptions kDefaultChainOptions = {
    10U, 10U, MessagePassingType::SHARED_PTR, MessagePassingType::SHARED_PTR
  };

  explicit PointCloud2FilterChainBase(
    RequiredInterfaces node_interfaces, const std::string& name = "pointcloud2_filter_chain",
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

  void PublishUnique(sensor_msgs::msg::PointCloud2::UniquePtr) override;

  void PublishShared(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) override;

  void PublishReference(const sensor_msgs::msg::PointCloud2& msg) override;

private:
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  rclcpp::Node::SharedPtr node_ptr_;
#endif
  bool default_best_effort_subscription_ {false};
  bool default_best_effort_publisher_ {false};
  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct_;
  std::unique_ptr<point_cloud_transport::TransportHints> transport_hints_;
  point_cloud_transport::Publisher pct_publisher_;
  point_cloud_transport::Subscriber pct_subscriber_;
};

}  // namespace sensor_filters
