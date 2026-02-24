// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for all sensor filter chains.
 */

#include <string>
#include <filters/filter_chain.hpp>
#include <rcl/rcl.h>

namespace sensor_filters {
    template <typename T>
    class FilterChainBase {
    protected:
        std::shared_ptr<rclcpp::Subscription<T>> inputSubscriber;
        std::shared_ptr<rclcpp::Publisher<T>> outputPublisher;
        rclcpp::Node::SharedPtr node;
        size_t inputQueueSize = 10u;
        size_t outputQueueSize = 10u;
        bool usePtrMessages = true;

        filters::FilterChain<T> filterChain;
        T msg;

    public:
        FilterChainBase() :
            filterChain(std::string(typeid(T).name())) {}

        virtual ~FilterChainBase() = default;

        virtual void initFilters(
            const std::string& filterChainNamespace,
            rclcpp::Node::SharedPtr node,
            const bool usePtrMessages,
            long inputQueueSize,
            long outputQueueSize
        ) {
            if (!this->filterChain.configure(filterChainNamespace, node->get_node_logging_interface(), node->get_node_parameters_interface())) {
                RCLCPP_ERROR_STREAM(node->get_logger(), "Configuration of filter chain for "
                                    << typeid(T).name() << " is invalid, the chain will not be run.");
                throw std::runtime_error("Filter configuration error");
            }

            RCLCPP_INFO_STREAM(node->get_logger(), "Configured filter chain of type " << typeid(T).name() << " from namespace "
                               << node->get_namespace() << "/"
                               << filterChainNamespace);

            this->node = node;
            this->outputQueueSize = outputQueueSize;
            this->inputQueueSize = inputQueueSize;
            this->usePtrMessages = usePtrMessages;

            this->advertise();
            this->subscribe();
        }

    protected:
        virtual void advertise() {
            this->outputPublisher = this->node->create_publisher<T>("output", this->outputQueueSize);
        }

        virtual void subscribe() {
            if (this->usePtrMessages) {
                this->inputSubscriber = this->node->template create_subscription<T>(
                    "input", this->inputQueueSize,
                    [this](const typename T::UniquePtr& msg) {
                        FilterChainBase::callbackUnique(msg);
                    }
                );
            } else {
                this->inputSubscriber = this->node->template create_subscription<T>(
                    "input", this->inputQueueSize,
                    [this](const T& msg) {
                        FilterChainBase::callbackReference(msg);
                    }
                );
            }
        }

        virtual void publishUnique(typename T::UniquePtr& msg) {
            this->outputPublisher->publish(std::move(msg));
        }

        virtual void publishShared(const typename T::ConstSharedPtr& msg) {
            RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "must be overriden by child class");
        }

        virtual void publishReference(const T& msg) {
            this->outputPublisher->publish(msg);
        }

        virtual void callbackUnique(const typename T::UniquePtr& msgIn) {
            typename T::UniquePtr msgOut = std::make_unique<T>();
            if (this->filter(*msgIn, *msgOut))
                this->publishUnique(msgOut);
        }

        virtual void callbackShared(const typename T::ConstSharedPtr& msgIn) {
            typename T::SharedPtr msgOut = std::make_shared<T>();
            if (this->filter(*msgIn, *msgOut))
                this->publishShared(msgOut);
        }

        virtual void callbackReference(const T& msgIn) {
            if (this->filter(msgIn, this->msg))
                this->publishReference(this->msg);
        }

        virtual bool filter(const T& msgIn, T& msgOut) {
            const auto clock = node->get_clock();
            const auto start = clock->now();
            if (!this->filterChain.update(msgIn, msgOut)) {
                RCLCPP_ERROR_THROTTLE(node->get_logger(), *clock, 1000, "Filtering data from time %i.%i failed.",
                                      msgIn.header.stamp.sec, msgIn.header.stamp.nanosec);
                return false;
            }
            RCLCPP_DEBUG_STREAM(node->get_logger(), "Filtering took " << (clock->now() - start).seconds() << " s.");
            return true;
        }
    };
}
