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
        explicit FilterChainNode(const std::string& name, const rclcpp::NodeOptions& options) :
            Node(name, options),
            Base(
                name,
                this->declare_parameter("input_queue_size", 10),
                this->declare_parameter("output_queue_size", 10),
                false,
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_parameters_interface(),
                this->get_node_logging_interface()
            ) {}

        //! use a composable node instead
        [[deprecated]] explicit FilterChainNode(const std::string& name) :
            Node(name),
            Base(
                name,
                this->declare_parameter("input_queue_size", 10),
                this->declare_parameter("output_queue_size", 10),
                false,
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_parameters_interface(),
                this->get_node_logging_interface()
            ) {}

        void configure() override {
            Base::configure();

            publish();
            subscribe();
        }

        virtual void publish() {
            this->outputPublisher = create_publisher<T>("output", this->outputQueueSize);
        }

        virtual void subscribe() {
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
        explicit LifecycleFilterChainNode(const std::string& name, const rclcpp::NodeOptions& options) :
            LifecycleNode(name, options),
            Base(
                name,
                this->declare_parameter("input_queue_size", 10),
                this->declare_parameter("output_queue_size", 10),
                false,
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_parameters_interface(),
                this->get_node_logging_interface()
            ) {}

        //! use a composable node instead
        [[deprecated]] explicit LifecycleFilterChainNode(const std::string& name) :
            LifecycleNode(name),
            Base(
                name,
                this->declare_parameter("input_queue_size", 10),
                this->declare_parameter("output_queue_size", 10),
                false,
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_parameters_interface(),
                this->get_node_logging_interface()
            ) {}

        CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
            advertise();
            subscribe();

            try {
                Base::configure();
            } catch (const std::runtime_error&) {
                return CallbackReturn::ERROR;
            }

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

        CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
            const auto result = LifecycleNode::on_activate(state);

            return result;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override {
            if (const auto result = LifecycleNode::on_deactivate(state); result != CallbackReturn::SUCCESS)
                return result;

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
            if (!this->outputPublisher->is_activated())
                return;

            this->outputPublisher->publish(std::move(msg));
        }

        void publishShared(const typename T::ConstSharedPtr&) override {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "must be overridden by child class");
        }

        void publishReference(const T& msg) override {
            if (!this->outputPublisher->is_activated())
                return;

            this->outputPublisher->publish(msg);
        }

    private:
        std::shared_ptr<rclcpp::Subscription<T>> inputSubscriber;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<T>> outputPublisher;
    };

    //! this method is deprecated, and instead a composable node should be used
    template <typename T, typename Base = FilterChainBase<T>>
    [[deprecated]] void spinFilterChain(const std::string& name, const int argc, char** argv) {
        rclcpp::init(argc, argv);
        const auto node = std::make_shared<FilterChainNode<T, Base>>(name);
        node->configure();
        rclcpp::spin(node);
    }

    //! this method is deprecated, and instead a composable node should be used
    template <typename T, typename Base = FilterChainBase<T>>
    [[deprecated]] void spinLifecycleFilterChain(const std::string& name, const int argc, char** argv) {
        rclcpp::init(argc, argv);
        const auto node = std::make_shared<LifecycleFilterChainNode<T, Base>>(name);
        rclcpp::spin(node);
    }
}
