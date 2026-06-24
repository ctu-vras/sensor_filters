// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_filters/FilterChainBase.hpp>

namespace sensor_filters {

FilterChainBaseGeneric::FilterChainBaseGeneric(
  RequiredInterfaces node_interfaces, std::string message_type, std::string filter_chain_namespace,
  const FilterChainOptions& default_options)
  : filter_chain_namespace_(std::move(filter_chain_namespace)), default_options_(default_options),
    options_(default_options), node_interfaces_(std::move(node_interfaces)), message_type_(std::move(message_type)) {
  const auto params = node_interfaces_.get_node_parameters_interface();

  params->declare_parameter(
    "input_queue_size", rclcpp::ParameterValue(static_cast<int64_t>(default_options.input_queue_size)));
  params->declare_parameter(
    "output_queue_size", rclcpp::ParameterValue(static_cast<int64_t>(default_options.output_queue_size)));
  params->declare_parameter(
    "subscription_type", rclcpp::ParameterValue(to_string(default_options.subscription_type)));
  params->declare_parameter(
    "publication_type", rclcpp::ParameterValue(to_string(default_options.publication_type)));
  params->declare_parameter(
    "is_lazy", rclcpp::ParameterValue(default_options.is_lazy));
  params->declare_parameter(
    "content_filter_expression", rclcpp::ParameterValue(std::string {}));
  params->declare_parameter(
    "content_filter_parameters", rclcpp::ParameterValue(std::vector<std::string> {}));
}

void FilterChainBaseGeneric::on_configure() {
  const auto logging_interface = node_interfaces_.get_node_logging_interface();
  const auto params_interface = node_interfaces_.get_node_parameters_interface();

  options_.input_queue_size = params_interface->get_parameter("input_queue_size").as_int();
  options_.output_queue_size = params_interface->get_parameter("output_queue_size").as_int();
  options_.is_lazy = params_interface->get_parameter("is_lazy").as_bool();

  options_.subscription_type = parseMessagePassingType(
    params_interface->get_parameter("subscription_type").as_string());
  if (!ValidateSubscriptionType()) {
    RCLCPP_FATAL(
      logging_interface->get_logger(),
      "Invalid subscription type %s", to_string(options_.subscription_type).c_str());
    throw std::runtime_error("Invalid subscription type " + to_string(options_.subscription_type));
  }

  options_.publication_type = parseMessagePassingType(params_interface->get_parameter("publication_type").as_string());
  if (!ValidatePublicationType()) {
    RCLCPP_FATAL(
      logging_interface->get_logger(),
      "Invalid publication type %s", to_string(options_.publication_type).c_str());
    throw std::runtime_error("Invalid publication type " + to_string(options_.publication_type));
  }

  publisher_options_.qos_overriding_options = kQosOverrides;
  if (options_.is_lazy) {
#ifndef MATCHED_EVENT_NOT_AVAILABLE
    publisher_options_.event_callbacks.matched_callback = [this](const rclcpp::MatchedInfo&) {
      std::lock_guard<std::mutex> lock(this->subscription_mutex_);
      if (this->GetNumSubscribers() == 0) {
        this->Unsubscribe();
        RCLCPP_INFO(
          this->node_interfaces_.get_node_logging_interface()->get_logger(),
          "Unsubscribed from lazy input topic");
      } else if (!this->IsSubscribed()) {
        this->Subscribe("input");
        RCLCPP_INFO(
          this->node_interfaces_.get_node_logging_interface()->get_logger(),
          "Subscribed to lazy input topic");
      }
    };
#else
    RCLCPP_ERROR(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "Lazy input topic is not available prior ROS 2 Iron.");
    options_.is_lazy = false;
#endif
  }

  subscription_options_.qos_overriding_options = kQosOverrides;
  subscription_options_.content_filter_options.filter_expression =
    params_interface->get_parameter("content_filter_expression").as_string();
  subscription_options_.content_filter_options.expression_parameters =
    params_interface->get_parameter("content_filter_parameters").as_string_array();

  on_configure_chain();

  Advertise("output");
  if (!options_.is_lazy) {
    Subscribe("input");
  }
}

void FilterChainBaseGeneric::on_cleanup() {
  {
    std::lock_guard<std::mutex> lock(subscription_mutex_);
    if (IsSubscribed()) {
      Unsubscribe();
    }
  }
  Unadvertise();

  options_ = default_options_;
  publisher_options_ = {};
  subscription_options_ = {};
}

void FilterChainBaseGeneric::on_shutdown() {
  on_cleanup();
}

bool FilterChainBaseGeneric::ValidateSubscriptionType() const {
  return false;
}

bool FilterChainBaseGeneric::ValidatePublicationType() const {
  return false;
}

} // namespace sensor_filters
