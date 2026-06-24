// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for all sensor filter chains.
 */

#include <algorithm>
#include <cassert>
#include <cctype>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include <filters/filter_chain.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/managed_entity.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sensor_filters/NodeInterfaces.hpp>

namespace sensor_filters {

enum class MessagePassingType {
  REFERENCE,
  SHARED_PTR,
  UNIQUE_PTR,
};

struct FilterChainOptions {
  size_t input_queue_size {10U};
  size_t output_queue_size {10U};
  MessagePassingType subscription_type {MessagePassingType::UNIQUE_PTR};
  MessagePassingType publication_type {MessagePassingType::UNIQUE_PTR};
  bool is_lazy {false};
};

inline MessagePassingType parseMessagePassingType(const std::string& type) {
  auto lower_type = type;
  std::transform(type.begin(), type.end(), lower_type.begin(), [](unsigned char c) { return std::tolower(c); });

  if (lower_type == "reference") {
    return MessagePassingType::REFERENCE;
  }
  if (lower_type == "shared_ptr") {
    return MessagePassingType::SHARED_PTR;
  }
  if (lower_type == "unique_ptr") {
    return MessagePassingType::UNIQUE_PTR;
  }
  throw std::invalid_argument("Invalid message passing type: " + type);
}

inline std::string to_string(const MessagePassingType type) {
  switch (type) {
    case MessagePassingType::REFERENCE:
      return "reference";
    case MessagePassingType::SHARED_PTR:
      return "shared_ptr";
    case MessagePassingType::UNIQUE_PTR:
      return "unique_ptr";
    default:
      assert(false && "Unexpected message passing type");
      return "unknown";
  }
}

template<typename, typename = void>
struct has_header : std::false_type {};

template<typename T>
struct has_header<T, std::void_t<decltype(std::declval<T>().header)>> : std::true_type {};

class FilterChainBaseGeneric : public rclcpp_lifecycle::SimpleManagedEntity {
protected:
  constexpr static std::initializer_list<rclcpp::QosPolicyKind> kQosOverrides = {
    rclcpp::QosPolicyKind::Deadline,
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Lifespan,
    rclcpp::QosPolicyKind::Liveliness,
    rclcpp::QosPolicyKind::LivelinessLeaseDuration,
    rclcpp::QosPolicyKind::Reliability,
  };

  std::string filter_chain_namespace_;
  const FilterChainOptions default_options_;
  FilterChainOptions options_;

  RequiredInterfaces node_interfaces_;
  rclcpp::PublisherOptions publisher_options_ {};
  rclcpp::SubscriptionOptions subscription_options_ {};

  std::string message_type_;

  rclcpp::Clock wall_clock_ {RCL_SYSTEM_TIME};

  std::mutex subscription_mutex_;

public:
  FilterChainBaseGeneric(
    RequiredInterfaces node_interfaces, std::string message_type, std::string filter_chain_namespace,
    const FilterChainOptions& default_options = {});

  virtual void on_configure();

  virtual void on_cleanup();

  virtual void on_shutdown();

protected:
  virtual void on_configure_chain() = 0;

  virtual void Advertise(const std::string& topic) = 0;

  virtual void Unadvertise() = 0;

  virtual void Subscribe(const std::string& topic) = 0;

  virtual void Unsubscribe() = 0;

  virtual bool IsSubscribed() const = 0;

  virtual size_t GetNumSubscribers() const = 0;

  virtual bool ValidateSubscriptionType() const;

  virtual bool ValidatePublicationType() const;
};

template<typename T>
class FilterChainBase : public FilterChainBaseGeneric {
public:
  typedef T Message;

protected:
  filters::FilterChain<T> filter_chain_;
  T cached_msg_;

public:
  FilterChainBase(
    RequiredInterfaces node_interfaces, std::string filter_chain_namespace,
    const FilterChainOptions& default_options = {})
    : FilterChainBaseGeneric(
        std::move(node_interfaces), rosidl_generator_traits::data_type<T>(), std::move(filter_chain_namespace),
        default_options),
      filter_chain_(message_type_) {}

  void on_cleanup() override {
    FilterChainBaseGeneric::on_cleanup();

    filter_chain_.clear();
  }

  void on_shutdown() override {
    on_cleanup();
  }

protected:
  void on_configure_chain() override {
    const auto logging_interface = node_interfaces_.get_node_logging_interface();
    const auto params_interface = node_interfaces_.get_node_parameters_interface();

    if (!filter_chain_.configure(filter_chain_namespace_, logging_interface, params_interface)) {
      RCLCPP_ERROR(
        logging_interface->get_logger(),
        "Configuration of filter chain for %s is invalid, the chain will not be run.", message_type_.c_str());
      throw std::runtime_error("Filter configuration error");
    }

    RCLCPP_INFO(
      logging_interface->get_logger(),
      "Filter chain %s configured with %lu filters.", filter_chain_namespace_.c_str(), filter_chain_.get_length());
  }

  virtual void PublishUnique(typename T::UniquePtr) {
    throw std::runtime_error("FilterChainBase does not support unique_ptr publications");
  }

  virtual void PublishShared(const typename T::ConstSharedPtr&) {
    throw std::runtime_error("FilterChainBase does not support shared_ptr publications");
  }

  virtual void PublishReference(const T&) {
    throw std::runtime_error("FilterChainBase does not support reference publications");
  }

  virtual void CallbackUnique(typename T::UniquePtr msg_in) {
    CallbackCommon(*msg_in);
  }

  virtual void CallbackShared(const typename T::ConstSharedPtr& msg_in) {
    CallbackCommon(*msg_in);
  }

  virtual void CallbackReference(const T& msg_in) {
    CallbackCommon(msg_in);
  }

  virtual void CallbackCommon(const T& msg_in) {
    if (!is_activated()) {
      return;
    }

    switch (options_.publication_type) {
      case MessagePassingType::UNIQUE_PTR:
      {
        auto msg_out = std::make_unique<T>();
        if (filter(msg_in, *msg_out)) {
          PublishUnique(std::move(msg_out));
        }
        break;
      }
      case MessagePassingType::SHARED_PTR:
      {
        auto msg_out = std::make_shared<T>();
        if (filter(msg_in, *msg_out)) {
          PublishShared(msg_out);
        }
        break;
      }
      case MessagePassingType::REFERENCE:
      {
        if (filter(msg_in, cached_msg_)) {
          PublishReference(cached_msg_);
        }
        break;
      }
      default:
        assert(false && "Unexpected publication type");
        break;
    }
  }

  virtual bool filter(const T& msg_in, T& msg_out) {
    const auto logging_interface = node_interfaces_.get_node_logging_interface();
    const auto start = wall_clock_.now();

    if (!filter_chain_.update(msg_in, msg_out)) {
      if constexpr (has_header<T>::value) {
        RCLCPP_ERROR_THROTTLE(
          logging_interface->get_logger(), wall_clock_, 1000,
          "Filtering data by filter %s at time %i.%09i failed.",
          filter_chain_namespace_.c_str(), msg_in.header.stamp.sec, msg_in.header.stamp.nanosec);
      } else {
        RCLCPP_ERROR_THROTTLE(
          logging_interface->get_logger(), wall_clock_, 1000,
          "Filtering data by filter %s failed.", filter_chain_namespace_.c_str());
      }
      return false;
    }
    const auto end = wall_clock_.now();
    RCLCPP_DEBUG(logging_interface->get_logger(), "Filtering took %0.09f s.", (end - start).seconds());
    return true;
  }
};

template<typename T>
class FilterChainNodeBase : public FilterChainBase<T> {
public:
  constexpr static FilterChainOptions kDefaultChainOptions = {};

  explicit FilterChainNodeBase(
    RequiredInterfaces node_interfaces, const std::string& name,
    const FilterChainOptions& default_chain_options = kDefaultChainOptions)
    : FilterChainBase<T>(node_interfaces, name, default_chain_options) {}

protected:
  void Advertise(const std::string& topic) override {
    output_publisher_ = rclcpp::create_publisher<T>(
      this->node_interfaces_, topic, rclcpp::QoS(this->options_.output_queue_size), this->publisher_options_);
  }

  void Unadvertise() override {
    output_publisher_.reset();
  }

  void Subscribe(const std::string& topic) override {
    switch (this->options_.subscription_type) {
      case MessagePassingType::UNIQUE_PTR:
      {
        input_subscriber_ = rclcpp::create_subscription<T>(
          this->node_interfaces_, topic, rclcpp::QoS(this->options_.input_queue_size),
          [this](typename T::UniquePtr msg) {
            FilterChainBase<T>::CallbackUnique(std::move(msg));
          },
          this->subscription_options_);
        break;
      }
      case MessagePassingType::SHARED_PTR:
      {
        input_subscriber_ = rclcpp::create_subscription<T>(
          this->node_interfaces_, topic, rclcpp::QoS(this->options_.input_queue_size),
          [this](const typename T::ConstSharedPtr& msg) {
            FilterChainBase<T>::CallbackShared(msg);
          },
          this->subscription_options_);
        break;
      }
      case MessagePassingType::REFERENCE:
      {
        input_subscriber_ = rclcpp::create_subscription<T>(
          this->node_interfaces_, topic, rclcpp::QoS(this->options_.input_queue_size),
          [this](const T& msg) {
            FilterChainBase<T>::CallbackReference(msg);
          },
          this->subscription_options_);
        break;
      }
      default:
        assert(false && "Unexpected subscription type");
    }
  }

  void Unsubscribe() override {
    input_subscriber_.reset();
  }

  bool IsSubscribed() const override {
    return input_subscriber_ != nullptr;
  }

  size_t GetNumSubscribers() const override {
    if (output_publisher_ == nullptr) {
      return 0u;
    }

    return output_publisher_->get_subscription_count() +
      output_publisher_->get_intra_process_subscription_count();
  }

  void PublishUnique(typename T::UniquePtr msg) override {
    if (!this->is_activated()) {
      return;
    }

    output_publisher_->publish(std::move(msg));
  }

  void PublishReference(const T& msg) override {
    if (!this->is_activated()) {
      return;
    }

    output_publisher_->publish(msg);
  }

  bool ValidateSubscriptionType() const override {
    return true;
  }

  bool ValidatePublicationType() const override {
    return this->options_.publication_type != MessagePassingType::SHARED_PTR;
  }

  typename rclcpp::Subscription<T>::SharedPtr input_subscriber_;
  typename rclcpp::Publisher<T>::SharedPtr output_publisher_;
};

} // namespace sensor_filters
