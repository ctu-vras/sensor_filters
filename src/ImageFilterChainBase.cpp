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
ImageFilterChainBase::ImageFilterChainBase(
  RequiredInterfaces node_interfaces, const std::string& name, const FilterChainOptions& default_chain_options)
  : FilterChainBase(node_interfaces, name, default_chain_options) {
  const auto params = node_interfaces.get_node_parameters_interface();
  // image_transport does not declare the parameter
  params->declare_parameter("image_transport", rclcpp::ParameterValue("raw"));
#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
  node_ptr_ = GetNodeSharedPtrFromInterfaces(node_interfaces);
  it_ = std::make_unique<image_transport::ImageTransport>(node_ptr_);
  transport_hints_ = std::make_unique<image_transport::TransportHints>(node_ptr_.get());
#else
  it_ = std::make_unique<image_transport::ImageTransport>(node_interfaces);
  transport_hints_ = std::make_unique<image_transport::TransportHints>(node_interfaces_);
#endif
}

void ImageFilterChainBase::on_configure() {
  FilterChainBase<sensor_msgs::msg::Image>::on_configure();

#ifdef IMAGE_TRANSPORT_PUB_OPTIONS_NOT_AVAILABLE
  if (options_.is_lazy) {
    RCLCPP_ERROR(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "Lazy input topic is not available for images in Humble.");
    options_.is_lazy = false;
    Subscribe("input");
  }
#endif
}

bool ImageFilterChainBase::ValidateSubscriptionType() const {
  return options_.subscription_type == MessagePassingType::SHARED_PTR;
}

bool ImageFilterChainBase::ValidatePublicationType() const {
  return true;
}

void ImageFilterChainBase::Advertise(const std::string& topic) {
  const auto topics = node_interfaces_.get_node_topics_interface();

#ifdef IMAGE_TRANSPORT_NODE_INTERFACES_NOT_AVAILABLE
#ifndef IMAGE_TRANSPORT_PUB_OPTIONS_NOT_AVAILABLE
  it_publisher_ = image_transport::create_publisher(
    node_ptr_.get(), topics->resolve_topic_name(topic), rclcpp::QoS(options_.output_queue_size).get_rmw_qos_profile(),
    publisher_options_);
#else
  it_publisher_ = image_transport::create_publisher(
    node_ptr_.get(), topics->resolve_topic_name(topic), rclcpp::QoS(options_.output_queue_size).get_rmw_qos_profile());
#endif
#else
  it_publisher_ = image_transport::create_publisher(
    node_interfaces_, topics->resolve_topic_name(topic), rclcpp::QoS(options_.output_queue_size), publisher_options_);
#endif
}

void ImageFilterChainBase::Unadvertise() {
  it_publisher_.shutdown();
}

void ImageFilterChainBase::Subscribe(const std::string& topic) {
  const auto topics = node_interfaces_.get_node_topics_interface();

  it_subscriber_ = it_->subscribe(
    topics->resolve_topic_name(topic), options_.input_queue_size,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
      this->CallbackShared(msg);
    },
    {}, transport_hints_.get(), subscription_options_);
}

void ImageFilterChainBase::Unsubscribe() {
  it_subscriber_.shutdown();
}

bool ImageFilterChainBase::IsSubscribed() const {
  return it_subscriber_;
}

size_t ImageFilterChainBase::GetNumSubscribers() const {
  return it_publisher_ ? it_publisher_.getNumSubscribers() : 0u;
}

void ImageFilterChainBase::PublishUnique(sensor_msgs::msg::Image::UniquePtr msg) {
  it_publisher_.publish(std::move(msg));
}

void ImageFilterChainBase::PublishShared(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  it_publisher_.publish(msg);
}

void ImageFilterChainBase::PublishReference(const sensor_msgs::msg::Image& msg) {
  it_publisher_.publish(msg);
}

} // namespace sensor_filters
