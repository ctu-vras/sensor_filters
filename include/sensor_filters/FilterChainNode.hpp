// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for a sensor filter chain node.
 */

#include <string>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_filters/FilterChainBase.hpp>

namespace sensor_filters {
    // TODO 2026-03-17 (solonovamax): maybe create a FilterChainNodeGeneric template that can be used for both normal & lifecycle nodes?

    template <typename T, typename Base = FilterChainBase<T>>
    class FilterChainNode : public rclcpp::Node, public Base {
    public:
        // TODO 2026-03-16 (solonovamax): support parameter callback
        explicit FilterChainNode(const std::string& messageType, const std::string& name, const rclcpp::NodeOptions& options) :
            Node(name, options),
            Base(
                messageType,
                name,
                this->declare_parameter("input_queue_size", 10),
                this->declare_parameter("output_queue_size", 10),
                false,
                *this
            ) {}

        void configure() override {
            Base::configure();

            advertise();
            subscribe();
        }

        void advertise() override {
            this->outputPublisher = create_publisher<T>("output", this->outputQueueSize);
        }

        void subscribe() override {
            if (this->usePtrMessages) {
                this->inputSubscriber = create_subscription<T>(
                    "input", this->inputQueueSize,
                    [this](const typename T::UniquePtr& msg) {
                        FilterChainBase<T>::callbackUnique(msg);
                    }
                );
            } else {
                this->inputSubscriber = create_subscription<T>(
                    "input", this->inputQueueSize,
                    [this](const T& msg) {
                        FilterChainBase<T>::callbackReference(msg);
                    }
                );
            }
        }

        bool isActive() override {
            return true;
        }

        void publishUnique(typename T::UniquePtr& msg) override {
            this->outputPublisher->publish(std::move(msg));
        }

        void publishShared(const typename T::ConstSharedPtr&) override {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "must be overridden by child class");
        }

        void publishReference(const T& msg) override {
            this->outputPublisher->publish(msg);
        }

    private:
        std::shared_ptr<rclcpp::Subscription<T>> inputSubscriber;
        std::shared_ptr<rclcpp::Publisher<T>> outputPublisher;
    };

    template <typename T, typename Base = FilterChainBase<T>>
    class LifecycleFilterChainNode : public rclcpp_lifecycle::LifecycleNode, public Base {
    public:
        // TODO 2026-03-16 (solonovamax): support parameter callback
        explicit LifecycleFilterChainNode(const std::string& messageType, const std::string& name, const rclcpp::NodeOptions& options) :
            LifecycleNode(name, options),
            Base(
                messageType,
                name,
                this->declare_parameter("input_queue_size", 10),
                this->declare_parameter("output_queue_size", 10),
                false,
                *this
            ) {}

        CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
            try {
                Base::configure();
            } catch (const std::runtime_error&) {
                return CallbackReturn::ERROR;
            }

            advertise();
            subscribe();

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
            // reset state to before on_configure()
            this->filterChain.clear();
            outputPublisher.reset();
            inputSubscriber.reset();

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
            this->filterChain.clear();

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_error(const rclcpp_lifecycle::State&) override {
            // currently an error can only occur in on_configure, so we don't need to check the previous state
            // if other transitions are ever changed so that they can error, then this needs to be updated.

            this->filterChain.clear();

            return CallbackReturn::SUCCESS;
        }

    protected:
        void advertise() override {
            this->outputPublisher = create_publisher<T>("output", this->outputQueueSize);
        }

        void subscribe() override {
            if (this->usePtrMessages) {
                this->inputSubscriber = create_subscription<T>(
                    "input", this->inputQueueSize,
                    [this](const typename T::UniquePtr& msg) {
                        FilterChainBase<T>::callbackUnique(msg);
                    }
                );
            } else {
                this->inputSubscriber = create_subscription<T>(
                    "input", this->inputQueueSize,
                    [this](const T& msg) {
                        FilterChainBase<T>::callbackReference(msg);
                    }
                );
            }
        }

        bool isActive() override {
            return this->outputPublisher->is_activated();
        }

        void publishUnique(typename T::UniquePtr& msg) override {
            if (!this->isActive())
                return;

            this->outputPublisher->publish(std::move(msg));
        }

        void publishShared(const typename T::ConstSharedPtr&) override {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "must be overridden by child class");
        }

        void publishReference(const T& msg) override {
            if (!this->isActive())
                return;

            this->outputPublisher->publish(msg);
        }

    private:
        std::shared_ptr<rclcpp::Subscription<T>> inputSubscriber;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<T>> outputPublisher;
    };

}
