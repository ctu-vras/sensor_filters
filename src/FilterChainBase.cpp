// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_filters/FilterChainBase.hpp>

namespace sensor_filters
{

FilterChainBaseGeneric::FilterChainBaseGeneric(RequiredInterfaces nodeInterfaces, std::string messageType,
  std::string filterChainNamespace, const FilterChainOptions& defaultOptions)
: filterChainNamespace(std::move(filterChainNamespace)), defaultOptions(defaultOptions), options(defaultOptions),
  nodeInterfaces(std::move(nodeInterfaces)), messageType(std::move(messageType))
{
  const auto params = this->nodeInterfaces.get_node_parameters_interface();
  params->declare_parameter(
    "input_queue_size", rclcpp::ParameterValue(static_cast<int64_t>(defaultOptions.inputQueueSize)));
  params->declare_parameter(
    "output_queue_size", rclcpp::ParameterValue(static_cast<int64_t>(defaultOptions.outputQueueSize)));
  params->declare_parameter(
    "subscription_type", rclcpp::ParameterValue(to_string(defaultOptions.subscriptionType)));
  params->declare_parameter(
    "publication_type", rclcpp::ParameterValue(to_string(defaultOptions.publicationType)));
  params->declare_parameter(
    "is_lazy", rclcpp::ParameterValue(defaultOptions.isLazy));
  params->declare_parameter(
    "content_filter_expression", rclcpp::ParameterValue(std::string{}));
  params->declare_parameter(
    "content_filter_parameters", rclcpp::ParameterValue(std::vector<std::string>{}));
}

void FilterChainBaseGeneric::on_configure()
{
  const auto loggingInterface = this->nodeInterfaces.get_node_logging_interface();
  const auto paramsInterface = this->nodeInterfaces.get_node_parameters_interface();

  this->options.inputQueueSize = paramsInterface->get_parameter("input_queue_size").as_int();
  this->options.outputQueueSize = paramsInterface->get_parameter("output_queue_size").as_int();
  this->options.isLazy = paramsInterface->get_parameter("is_lazy").as_bool();

  this->options.subscriptionType = parseMessagePassingType(
    paramsInterface->get_parameter("subscription_type").as_string());
  if (!this->validateSubscriptionType())
  {
    RCLCPP_FATAL(loggingInterface->get_logger(),
                 "Invalid subscription type %s", to_string(this->options.subscriptionType).c_str());
    throw std::runtime_error("Invalid subscription type " + to_string(this->options.subscriptionType));
  }

  this->options.publicationType = parseMessagePassingType(
    paramsInterface->get_parameter("publication_type").as_string());
  if (!this->validatePublicationType())
  {
    RCLCPP_FATAL(loggingInterface->get_logger(),
                 "Invalid publication type %s", to_string(this->options.publicationType).c_str());
    throw std::runtime_error("Invalid publication type " + to_string(this->options.publicationType));
  }

  this->publisherOptions.qos_overriding_options = this->qosOverrides;
  if (this->options.isLazy) {
#ifndef MATCHED_EVENT_NOT_AVAILABLE
    this->publisherOptions.event_callbacks.matched_callback = [this](const rclcpp::MatchedInfo&) {
      std::lock_guard<std::mutex> lock(this->subscriptionMutex);
      if (this->getNumSubscribers() == 0)
      {
        this->unsubscribe();
        RCLCPP_INFO(this->nodeInterfaces.get_node_logging_interface()->get_logger(),
          "Unsubscribed from lazy input topic");
      }
      else if (!this->isSubscribed())
      {
        this->subscribe("input");
        RCLCPP_INFO(this->nodeInterfaces.get_node_logging_interface()->get_logger(), "Subscribed to lazy input topic");
      }
    };
#else
    RCLCPP_ERROR(this->nodeInterfaces.get_node_logging_interface()->get_logger(),
      "Lazy input topic is not available prior ROS 2 Iron.");
    this->options.isLazy = false;
#endif
  }

  this->subscriptionOptions.qos_overriding_options = this->qosOverrides;
  this->subscriptionOptions.content_filter_options.filter_expression =
    paramsInterface->get_parameter("content_filter_expression").as_string();
  this->subscriptionOptions.content_filter_options.expression_parameters =
    paramsInterface->get_parameter("content_filter_parameters").as_string_array();

  this->on_configure_chain();

  this->advertise("output");
  if (!this->options.isLazy)
    this->subscribe("input");
}

void FilterChainBaseGeneric::on_cleanup()
{
  {
    std::lock_guard<std::mutex> lock(this->subscriptionMutex);
    if (this->isSubscribed())
      this->unsubscribe();
  }
  this->unadvertise();

  this->options = this->defaultOptions;
  this->publisherOptions = {};
  this->subscriptionOptions = {};
}

void FilterChainBaseGeneric::on_shutdown()
{
  this->on_cleanup();
}

bool FilterChainBaseGeneric::validateSubscriptionType() const
{
  return false;
}

bool FilterChainBaseGeneric::validatePublicationType() const
{
  return false;
}

} // namespace sensor_filters
