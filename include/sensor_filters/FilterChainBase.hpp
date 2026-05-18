// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for all sensor filter chains.
 */

#include <string>
#include <utility>

#include <filters/filter_chain.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/managed_entity.hpp>

namespace sensor_filters {

    using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
        rclcpp::node_interfaces::NodeBaseInterface,
        rclcpp::node_interfaces::NodeParametersInterface,
        rclcpp::node_interfaces::NodeLoggingInterface,
        rclcpp::node_interfaces::NodeTopicsInterface
    >;

    enum class MessagePassingType {
        REFERENCE,
        SHARED_PTR,
        UNIQUE_PTR,
    };

    struct FilterChainOptions {
        size_t inputQueueSize {10U};
        size_t outputQueueSize {10U};
        MessagePassingType subscriptionType {MessagePassingType::UNIQUE_PTR};
        MessagePassingType publicationType {MessagePassingType::UNIQUE_PTR};
    };

    MessagePassingType parseMessagePassingType(const std::string& type)
    {
        auto lowerType = type;
        std::transform(type.begin(), type.end(), lowerType.begin(), [](unsigned char c){ return std::tolower(c); });

        if (lowerType == "reference")
            return MessagePassingType::REFERENCE;
        if (lowerType == "shared_ptr")
            return MessagePassingType::SHARED_PTR;
        if (lowerType == "unique_ptr")
            return MessagePassingType::UNIQUE_PTR;
        throw std::invalid_argument("Invalid message passing type: " + type);
    }

    std::string to_string(const MessagePassingType type)
    {
        switch (type)
        {
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

    template <typename T>
    class FilterChainBase : public rclcpp_lifecycle::SimpleManagedEntity {

    protected:
        std::string filterChainNamespace;
        const FilterChainOptions defaultOptions;
        FilterChainOptions options;

        RequiredInterfaces nodeInterfaces;

        filters::FilterChain<T> filterChain;
        std::string messageType;
        T cachedMsg;

        rclcpp::Clock wallClock {RCL_SYSTEM_TIME};

        std::shared_ptr<rclcpp::Subscription<T>> inputSubscriber;
        std::shared_ptr<rclcpp::Publisher<T>> outputPublisher;

    public:
        FilterChainBase(
            RequiredInterfaces nodeInterfaces,
            const std::string& messageType,
            std::string filterChainNamespace,
            const FilterChainOptions& defaultOptions = {}
        ) : filterChainNamespace(std::move(filterChainNamespace)),
            defaultOptions(defaultOptions), options(defaultOptions),
            nodeInterfaces(std::move(nodeInterfaces)), filterChain(messageType), messageType(messageType)
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
        }

        virtual void configure() {
            const auto loggingInterface = this->nodeInterfaces.get_node_logging_interface();
            const auto paramsInterface = this->nodeInterfaces.get_node_parameters_interface();

            this->options.inputQueueSize = paramsInterface->get_parameter("input_queue_size").as_int();
            this->options.outputQueueSize = paramsInterface->get_parameter("output_queue_size").as_int();

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

            if (!this->filterChain.configure(filterChainNamespace, loggingInterface, paramsInterface)) {
                RCLCPP_ERROR_STREAM(loggingInterface->get_logger(), "Configuration of filter chain for "
                                    << messageType << " is invalid, the chain will not be run.");
                throw std::runtime_error("Filter configuration error");
            }
        }

    protected:
        virtual void advertise()
        {
            this->outputPublisher = rclcpp::create_publisher<T>(this->nodeInterfaces, "output", rclcpp::QoS(this->options.outputQueueSize));
        }

        virtual void subscribe()
        {
            switch (this->options.subscriptionType) {
                case MessagePassingType::UNIQUE_PTR:
                {

                    this->inputSubscriber = rclcpp::create_subscription<T>(
                        this->nodeInterfaces, "input", rclcpp::QoS(this->options.inputQueueSize),
                        [this](typename T::UniquePtr msg) {
                            FilterChainBase<T>::callbackUnique(std::move(msg));
                        }
                    );
                    break;
                }
                case MessagePassingType::SHARED_PTR:
                {
                    this->inputSubscriber = rclcpp::create_subscription<T>(
                        this->nodeInterfaces, "input", rclcpp::QoS(this->options.inputQueueSize),
                        [this](const typename T::ConstSharedPtr& msg) {
                            FilterChainBase<T>::callbackShared(msg);
                        }
                    );
                    break;
                }
                case MessagePassingType::REFERENCE:
                {
                    this->inputSubscriber = rclcpp::create_subscription<T>(
                        this->nodeInterfaces, "input", rclcpp::QoS(this->options.inputQueueSize),
                        [this](const T& msg) {
                            FilterChainBase<T>::callbackReference(msg);
                        });
                    break;
                }
                default:
                    assert(false && "Unexpected subscription type");
            }
        }

        virtual void publishUnique(typename T::UniquePtr msg)
        {
            if (!this->is_activated())
                return;

            this->outputPublisher->publish(std::move(msg));
        }

        virtual void publishShared(const typename T::ConstSharedPtr&)
        {
            throw std::runtime_error("FilterChainBase does not support shared_ptr publications");
        }

        virtual void publishReference(const T& msg)
        {
            if (!this->is_activated())
                return;

            this->outputPublisher->publish(msg);
        }

        virtual bool validateSubscriptionType() const
        {
            return true;
        }

        virtual bool validatePublicationType() const
        {
            return this->options.publicationType != MessagePassingType::SHARED_PTR;
        }

        virtual void callbackUnique(typename T::UniquePtr msgIn) {
            this->callbackCommon(*msgIn);
        }

        virtual void callbackShared(const typename T::ConstSharedPtr& msgIn) {
            this->callbackCommon(*msgIn);
        }

        virtual void callbackReference(const T& msgIn) {
            this->callbackCommon(msgIn);
        }

        virtual void callbackCommon(const T& msgIn) {
            if (!this->is_activated())
                return;

            switch (this->options.publicationType)
            {
                case MessagePassingType::UNIQUE_PTR:
                {
                    auto msgOut = std::make_unique<T>();
                    if (this->filter(msgIn, *msgOut))
                        this->publishUnique(std::move(msgOut));
                    break;
                }
                case MessagePassingType::SHARED_PTR:
                {
                    auto msgOut = std::make_shared<T>();
                    if (this->filter(msgIn, *msgOut))
                        this->publishShared(msgOut);
                    break;
                }
                case MessagePassingType::REFERENCE:
                {
                    if (this->filter(msgIn, this->cachedMsg))
                        this->publishReference(this->cachedMsg);
                    break;
                }
                default:
                    assert(false && "Unexpected publication type");
                    break;
            }
        }

        virtual bool filter(const T& msgIn, T& msgOut) {
            const auto loggingInterface = this->nodeInterfaces.get_node_logging_interface();
            const auto start = this->wallClock.now();
            if (!this->filterChain.update(msgIn, msgOut)) {
                RCLCPP_ERROR_THROTTLE(loggingInterface->get_logger(), this->wallClock, 1000, "Filtering data from time %i.%09i failed.",
                                      msgIn.header.stamp.sec, msgIn.header.stamp.nanosec);
                return false;
            }
            const auto end = this->wallClock.now();
            RCLCPP_DEBUG(loggingInterface->get_logger(), "Filtering took %0.09f s.", (end - start).seconds());
            return true;
        }
    };
}
