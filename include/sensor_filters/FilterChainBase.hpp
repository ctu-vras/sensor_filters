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

namespace sensor_filters {

    using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
        rclcpp::node_interfaces::NodeBaseInterface,
        rclcpp::node_interfaces::NodeParametersInterface,
        rclcpp::node_interfaces::NodeLoggingInterface
    >;

    template <typename T>
    class FilterChainBase {

    protected:
        std::string filterChainNamespace;
        size_t inputQueueSize = 10u;
        size_t outputQueueSize = 10u;
        bool usePtrMessages = true;

        RequiredInterfaces nodeInterfaces;

        filters::FilterChain<T> filterChain;
        std::string messageType;
        T msg;

        rclcpp::Clock wallClock {RCL_SYSTEM_TIME};

    public:
        FilterChainBase(
            const std::string& messageType,
            std::string filterChainNamespace,
            const long inputQueueSize,
            const long outputQueueSize,
            const bool usePtrMessages,
            RequiredInterfaces nodeInterfaces
        ) : filterChainNamespace(std::move(filterChainNamespace)), inputQueueSize(inputQueueSize), outputQueueSize(outputQueueSize),
            usePtrMessages(usePtrMessages), nodeInterfaces(std::move(nodeInterfaces)), filterChain(messageType),
            messageType(messageType)
        {
        }

        virtual ~FilterChainBase() = default;

        virtual void configure() {
            const auto loggingInterface = this->nodeInterfaces.get_node_logging_interface();
            const auto paramsInterface = this->nodeInterfaces.get_node_parameters_interface();
            if (!this->filterChain.configure(filterChainNamespace, loggingInterface, paramsInterface)) {
                RCLCPP_ERROR_STREAM(loggingInterface->get_logger(), "Configuration of filter chain for "
                                    << messageType << " is invalid, the chain will not be run.");
                throw std::runtime_error("Filter configuration error");
            }
        }

    protected:
        virtual void advertise() = 0;

        virtual void subscribe() = 0;

        virtual bool isActive() = 0;

        virtual void publishUnique(typename T::UniquePtr& msg) = 0;

        virtual void publishShared(const typename T::ConstSharedPtr& msg) = 0;

        virtual void publishReference(const T& msg) = 0;

        virtual void callbackUnique(const typename T::UniquePtr& msgIn) {
            if (!isActive())
                return;

            typename T::UniquePtr msgOut = std::make_unique<T>();
            if (this->filter(*msgIn, *msgOut))
                this->publishUnique(msgOut);
        }

        virtual void callbackShared(const typename T::ConstSharedPtr& msgIn) {
            if (!isActive())
                return;

            typename T::SharedPtr msgOut = std::make_shared<T>();
            if (this->filter(*msgIn, *msgOut))
                this->publishShared(msgOut);
        }

        virtual void callbackReference(const T& msgIn) {
            if (!isActive())
                return;

            if (this->filter(msgIn, this->msg))
                this->publishReference(this->msg);
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
