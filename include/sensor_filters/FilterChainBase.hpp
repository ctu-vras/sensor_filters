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
#include <vector>

#include <filters/filter_chain.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/clock.hpp>
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
        size_t inputQueueSize {10U};
        size_t outputQueueSize {10U};
        MessagePassingType subscriptionType {MessagePassingType::UNIQUE_PTR};
        MessagePassingType publicationType {MessagePassingType::UNIQUE_PTR};
        bool isLazy {false};
    };

    inline MessagePassingType parseMessagePassingType(const std::string& type)
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

    inline std::string to_string(const MessagePassingType type)
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

    template <typename, typename = void>
    struct has_header : std::false_type {};

    template <typename T>
    struct has_header<T, std::void_t<decltype(std::declval<T>().header)>> : std::true_type {};

    class FilterChainBaseGeneric : public rclcpp_lifecycle::SimpleManagedEntity {

    protected:
        constexpr static std::initializer_list<rclcpp::QosPolicyKind> qosOverrides = {
            rclcpp::QosPolicyKind::Deadline,
            rclcpp::QosPolicyKind::Depth,
            rclcpp::QosPolicyKind::Durability,
            rclcpp::QosPolicyKind::History,
            rclcpp::QosPolicyKind::Lifespan,
            rclcpp::QosPolicyKind::Liveliness,
            rclcpp::QosPolicyKind::LivelinessLeaseDuration,
            rclcpp::QosPolicyKind::Reliability,
        };

        std::string filterChainNamespace;
        const FilterChainOptions defaultOptions;
        FilterChainOptions options;

        RequiredInterfaces nodeInterfaces;
        rclcpp::PublisherOptions publisherOptions {};
        rclcpp::SubscriptionOptions subscriptionOptions {};

        std::string messageType;

        rclcpp::Clock wallClock {RCL_SYSTEM_TIME};

        std::mutex subscriptionMutex;

    public:
        FilterChainBaseGeneric(RequiredInterfaces nodeInterfaces, std::string messageType,
            std::string filterChainNamespace, const FilterChainOptions& defaultOptions = {});

        virtual void on_configure();

        virtual void on_cleanup();

        virtual void on_shutdown();

    protected:
        virtual void on_configure_chain() = 0;

        virtual void advertise(const std::string& topic) = 0;

        virtual void unadvertise() = 0;

        virtual void subscribe(const std::string& topic) = 0;

        virtual void unsubscribe() = 0;

        virtual bool isSubscribed() const = 0;

        virtual size_t getNumSubscribers() const = 0;

        virtual bool validateSubscriptionType() const;

        virtual bool validatePublicationType() const;
    };

    template <typename T>
    class FilterChainBase : public FilterChainBaseGeneric {

    public:
        typedef T Message;

    protected:
        filters::FilterChain<T> filterChain;
        T cachedMsg;

    public:
        FilterChainBase(
            RequiredInterfaces nodeInterfaces,
            std::string filterChainNamespace,
            const FilterChainOptions& defaultOptions = {}
        ) : FilterChainBaseGeneric(std::move(nodeInterfaces), rosidl_generator_traits::data_type<T>(),
            std::move(filterChainNamespace), defaultOptions), filterChain(this->messageType)
        {
        }

        void on_cleanup() override {
            FilterChainBaseGeneric::on_cleanup();

            this->filterChain.clear();
        }

        void on_shutdown() override {
            this->on_cleanup();
        }

    protected:
        void on_configure_chain() override {
            const auto loggingInterface = this->nodeInterfaces.get_node_logging_interface();
            const auto paramsInterface = this->nodeInterfaces.get_node_parameters_interface();

            if (!this->filterChain.configure(filterChainNamespace, loggingInterface, paramsInterface)) {
                RCLCPP_ERROR_STREAM(loggingInterface->get_logger(), "Configuration of filter chain for "
                                    << messageType << " is invalid, the chain will not be run.");
                throw std::runtime_error("Filter configuration error");
            }

            RCLCPP_INFO(loggingInterface->get_logger(), "Filter chain %s configured with %lu filters.",
                filterChainNamespace.c_str(), this->filterChain.get_length());
        }

        virtual void publishUnique(typename T::UniquePtr)
        {
            throw std::runtime_error("FilterChainBase does not support unique_ptr publications");
        }

        virtual void publishShared(const typename T::ConstSharedPtr&)
        {
            throw std::runtime_error("FilterChainBase does not support shared_ptr publications");
        }

        virtual void publishReference(const T&)
        {
            throw std::runtime_error("FilterChainBase does not support reference publications");
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
                if constexpr (has_header<T>::value) {
                    RCLCPP_ERROR_THROTTLE(loggingInterface->get_logger(), this->wallClock, 1000,
                        "Filtering data by filter %s at time %i.%09i failed.", this->filterChainNamespace.c_str(),
                        msgIn.header.stamp.sec, msgIn.header.stamp.nanosec);
                }
                else {
                    RCLCPP_ERROR_THROTTLE(loggingInterface->get_logger(), this->wallClock, 1000,
                        "Filtering data by filter %s failed.", this->filterChainNamespace.c_str());
                }
                return false;
            }
            const auto end = this->wallClock.now();
            RCLCPP_DEBUG(loggingInterface->get_logger(), "Filtering took %0.09f s.", (end - start).seconds());
            return true;
        }
    };

    template <typename T>
    class FilterChainNodeBase : public FilterChainBase<T> {
    public:
        constexpr static FilterChainOptions DEFAULT_CHAIN_OPTIONS = {};

        explicit FilterChainNodeBase(RequiredInterfaces nodeInterfaces, const std::string& name,
            const FilterChainOptions& defaultChainOptions = DEFAULT_CHAIN_OPTIONS) :
            FilterChainBase<T>(nodeInterfaces, name, defaultChainOptions)
        {
        }

    protected:
        void advertise(const std::string& topic) override
        {
            this->outputPublisher = rclcpp::create_publisher<T>(
                this->nodeInterfaces, topic, rclcpp::QoS(this->options.outputQueueSize), this->publisherOptions);
        }

        void unadvertise() override
        {
            this->outputPublisher.reset();
        }

        void subscribe(const std::string& topic) override
        {
            switch (this->options.subscriptionType) {
                case MessagePassingType::UNIQUE_PTR:
                {

                    this->inputSubscriber = rclcpp::create_subscription<T>(
                        this->nodeInterfaces, topic, rclcpp::QoS(this->options.inputQueueSize),
                        [this](typename T::UniquePtr msg) {
                            FilterChainBase<T>::callbackUnique(std::move(msg));
                        }, this->subscriptionOptions);
                    break;
                }
                case MessagePassingType::SHARED_PTR:
                {
                    this->inputSubscriber = rclcpp::create_subscription<T>(
                        this->nodeInterfaces, topic, rclcpp::QoS(this->options.inputQueueSize),
                        [this](const typename T::ConstSharedPtr& msg) {
                            FilterChainBase<T>::callbackShared(msg);
                        }, this->subscriptionOptions);
                    break;
                }
                case MessagePassingType::REFERENCE:
                {
                    this->inputSubscriber = rclcpp::create_subscription<T>(
                        this->nodeInterfaces, topic, rclcpp::QoS(this->options.inputQueueSize),
                        [this](const T& msg) {
                            FilterChainBase<T>::callbackReference(msg);
                        }, this->subscriptionOptions);
                    break;
                }
                default:
                    assert(false && "Unexpected subscription type");
            }
        }

        void unsubscribe() override
        {
            this->inputSubscriber.reset();
        }

        bool isSubscribed() const override
        {
            return this->inputSubscriber != nullptr;
        }

        size_t getNumSubscribers() const override
        {
            if (this->outputPublisher == nullptr)
                return 0u;

            return this->outputPublisher->get_subscription_count() +
                this->outputPublisher->get_intra_process_subscription_count();
        }

        void publishUnique(typename T::UniquePtr msg) override
        {
            if (!this->is_activated())
                return;

            this->outputPublisher->publish(std::move(msg));
        }

        void publishReference(const T& msg) override
        {
            if (!this->is_activated())
                return;

            this->outputPublisher->publish(msg);
        }

        bool validateSubscriptionType() const override
        {
            return true;
        }

        bool validatePublicationType() const override
        {
            return this->options.publicationType != MessagePassingType::SHARED_PTR;
        }

        typename rclcpp::Subscription<T>::SharedPtr inputSubscriber;
        typename rclcpp::Publisher<T>::SharedPtr outputPublisher;
    };
}
