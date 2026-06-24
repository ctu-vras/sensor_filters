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

PointCloud2FilterChainBase::PointCloud2FilterChainBase(
  RequiredInterfaces node_interfaces, const std::string& name, const FilterChainOptions& default_chain_options)
  : FilterChainBase(node_interfaces, name, default_chain_options) {
#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  node_ptr_ = GetNodeSharedPtrFromInterfaces(node_interfaces);
  pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>(node_ptr_);
  transport_hints_ = std::make_unique<point_cloud_transport::TransportHints>(node_ptr_);
#else
  pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>(node_interfaces);
  transport_hints_ = std::make_unique<point_cloud_transport::TransportHints>(node_interfaces_);
#endif

  const auto params = node_interfaces_.get_node_parameters_interface();

  params->declare_parameter("default_best_effort_subscription", rclcpp::ParameterValue(false));
  params->declare_parameter("default_best_effort_publisher", rclcpp::ParameterValue(false));
}

void PointCloud2FilterChainBase::on_configure() {
  const auto params_interface = node_interfaces_.get_node_parameters_interface();

  default_best_effort_subscription_ = params_interface->get_parameter("default_best_effort_subscription").as_bool();
  default_best_effort_publisher_ = params_interface->get_parameter("default_best_effort_publisher").as_bool();

  FilterChainBase<sensor_msgs::msg::PointCloud2_<std::allocator<void>>>::on_configure();
}

bool PointCloud2FilterChainBase::ValidateSubscriptionType() const {
  return options_.subscription_type == MessagePassingType::SHARED_PTR;
}

bool PointCloud2FilterChainBase::ValidatePublicationType() const {
  return options_.publication_type != MessagePassingType::UNIQUE_PTR;
}

void PointCloud2FilterChainBase::Advertise(const std::string& topic) {
  const auto topics = node_interfaces_.get_node_topics_interface();

  rclcpp::QoS qos(options_.output_queue_size);
  if (default_best_effort_publisher_) {
    qos = rclcpp::SensorDataQoS().keep_last(options_.output_queue_size);
  }

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  pct_publisher_ = pct_->advertise(topics->resolve_topic_name(topic), qos.get_rmw_qos_profile(), publisher_options_);
#else
  pct_publisher_ = pct_->advertise(topics->resolve_topic_name(topic), qos, publisher_options_);
#endif
}

void PointCloud2FilterChainBase::Unadvertise() {
  pct_publisher_.shutdown();
}

void PointCloud2FilterChainBase::Subscribe(const std::string& topic) {
  const auto topics = node_interfaces_.get_node_topics_interface();

  rclcpp::QoS qos(options_.input_queue_size);
  if (default_best_effort_subscription_) {
    qos = rclcpp::SensorDataQoS().keep_last(options_.input_queue_size);
  }

#ifdef POINT_CLOUD_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  pct_subscriber_ = point_cloud_transport::create_subscription(
    node_ptr_, topics->resolve_topic_name(topic),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
      this->CallbackShared(msg);
    },
    pct_->getTransportOrDefault(transport_hints_.get()), qos.get_rmw_qos_profile(), subscription_options_);
#else
  pct_subscriber_ = pct_->subscribe(
    topics->resolve_topic_name(topic), qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
      this->CallbackShared(msg);
    },
    {}, transport_hints_.get(), subscription_options_);
#endif
}

void PointCloud2FilterChainBase::Unsubscribe() {
  pct_subscriber_.shutdown();
}

bool PointCloud2FilterChainBase::IsSubscribed() const {
  return pct_subscriber_;
}

size_t PointCloud2FilterChainBase::GetNumSubscribers() const {
  if (!pct_publisher_) {
    return 0u;
  }

  size_t count {0u};
  for (const auto& [topic, pub] : pct_publisher_.getPublishers()) {
    count += pub->get_subscription_count() + pub->get_subscription_count();
  }
  return count;
}

void PointCloud2FilterChainBase::PublishUnique(sensor_msgs::msg::PointCloud2::UniquePtr) {
  throw std::runtime_error("PointCloud2FilterChainNode does not support unique_ptr publications");
}

void PointCloud2FilterChainBase::PublishShared(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  pct_publisher_.publish(msg);
}

void PointCloud2FilterChainBase::PublishReference(const sensor_msgs::msg::PointCloud2& msg) {
  pct_publisher_.publish(msg);
}

}  // namespace sensor_filters
