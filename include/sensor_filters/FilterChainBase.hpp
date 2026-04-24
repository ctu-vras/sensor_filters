// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for all sensor filter chains.
 */

#include <string>
#include <filters/filter_chain.hpp>
#include <utility>

#if defined(__GNUC__) || defined(__clang__)
#include <cxxabi.h>
#endif

std::string demangle_cpp_type_name(const char * mangled_name)
{
#if defined(__GNUC__) || defined(__clang__)
  int status = 0;
  char * d = abi::__cxa_demangle(mangled_name, nullptr, nullptr, &status);
  std::string result = (status == 0 && d) ? d : mangled_name;
  std::free(d);
  return result;
#elif defined(_MSC_VER)
  // MSVC's typeid().name() is already human-readable, but prepends 'class ' or 'struct '
  std::string result = mangled_name;
  if (result.size() > 6 && result.substr(0, 6) == "class ") {
    result = result.substr(6);
  }
  if (result.size() > 7 && result.substr(0, 7) == "struct ") {
    result = result.substr(7);
  }
  return result;
#else
  return mangled_name;
#endif
}

namespace sensor_filters {
    template <typename T>
    class FilterChainBase {
    protected:
        std::string filterChainNamespace;
        size_t inputQueueSize = 10u;
        size_t outputQueueSize = 10u;
        bool usePtrMessages = true;

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr baseInterface;
        rclcpp::node_interfaces::NodeClockInterface::SharedPtr clockInterface;
        rclcpp::node_interfaces::NodeParametersInterface::SharedPtr paramsInterface;
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr loggingInterface;

        filters::FilterChain<T> filterChain;
        T msg;

    public:
        FilterChainBase(
            std::string filterChainNamespace,
            const long inputQueueSize,
            const long outputQueueSize,
            const bool usePtrMessages,
            rclcpp::node_interfaces::NodeBaseInterface::SharedPtr baseInterface,
            rclcpp::node_interfaces::NodeClockInterface::SharedPtr clockInterface,
            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr paramsInterface,
            rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr loggingInterface
        ) : filterChainNamespace(std::move(filterChainNamespace)), inputQueueSize(inputQueueSize), outputQueueSize(outputQueueSize),
            usePtrMessages(usePtrMessages), baseInterface(std::move(baseInterface)), clockInterface(std::move(clockInterface)),
            paramsInterface(std::move(paramsInterface)), loggingInterface(std::move(loggingInterface)), filterChain(demangle_cpp_type_name(typeid(T).name())) {}

        virtual ~FilterChainBase() = default;

        virtual void configure() {
            if (!this->filterChain.configure(filterChainNamespace, loggingInterface, paramsInterface)) {
                RCLCPP_ERROR_STREAM(loggingInterface->get_logger(), "Configuration of filter chain for "
                                    << typeid(T).name() << " is invalid, the chain will not be run.");
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
            const auto clock = clockInterface->get_clock();
            const auto start = clock->now();
            if (!this->filterChain.update(msgIn, msgOut)) {
                RCLCPP_ERROR_THROTTLE(loggingInterface->get_logger(), *clock, 1000, "Filtering data from time %i.%i failed.",
                                      msgIn.header.stamp.sec, msgIn.header.stamp.nanosec);
                return false;
            }
            RCLCPP_DEBUG_STREAM(loggingInterface->get_logger(), "Filtering took " << (clock->now() - start).seconds() << " s.");
            return true;
        }
    };
}
