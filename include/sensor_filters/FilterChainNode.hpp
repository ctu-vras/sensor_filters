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
        explicit FilterChainNode(const std::string& messageType, const std::string& name,
            const rclcpp::NodeOptions& options, const FilterChainOptions& defaultChainOptions = {}) :
            Node(name, options), Base(*this, messageType, name, defaultChainOptions)
        {
        }

        ~FilterChainNode() override
        {
            Base::on_deactivate();
        }

        void configure() override {
            Base::configure();
            Base::on_activate();

            advertise();
            subscribe();
        }

        bool validatePublicationType() const override {
            return this->options.publicationType != MessagePassingType::SHARED_PTR;
        }

        void advertise() override {
            this->outputPublisher = create_publisher<T>("output", this->options.outputQueueSize);
        }

        void subscribe() override {
            switch (this->options.subscriptionType) {
                case MessagePassingType::UNIQUE_PTR:
                {
                    this->inputSubscriber = create_subscription<T>(
                        "input", this->options.inputQueueSize,
                        [this](typename T::UniquePtr msg) {
                            FilterChainBase<T>::callbackUnique(std::move(msg));
                        }
                    );
                    break;
                }
                case MessagePassingType::SHARED_PTR:
                {
                    this->inputSubscriber = create_subscription<T>(
                        "input", this->options.inputQueueSize,
                        [this](const typename T::ConstSharedPtr& msg) {
                            FilterChainBase<T>::callbackShared(msg);
                        }
                    );
                    break;
                }
                case MessagePassingType::REFERENCE:
                {
                    this->inputSubscriber = create_subscription<T>(
                        "input", this->options.inputQueueSize,
                        [this](const T& msg) {
                            FilterChainBase<T>::callbackReference(msg);
                        });
                    break;
                }
                default:
                    assert(false && "Unexpected subscription type");
            }
        }

        void publishUnique(typename T::UniquePtr msg) override {
            this->outputPublisher->publish(std::move(msg));
        }

        void publishShared(const typename T::ConstSharedPtr&) override {
            throw std::runtime_error("FilterChainNode does not support shared_ptr publications");
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
        explicit LifecycleFilterChainNode(const std::string& messageType, const std::string& name,
            const rclcpp::NodeOptions& options, const FilterChainOptions& defaultChainOptions = {}) :
            LifecycleNode(name, options), Base(*this, messageType, name, defaultChainOptions)
        {
        }

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

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override
        {
            Base::on_activate();
            return rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override
        {
            Base::on_deactivate();
            return rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
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
        bool validatePublicationType() const override {
            return this->options.publicationType != MessagePassingType::SHARED_PTR;
        }

        void advertise() override {
            this->outputPublisher = create_publisher<T>("output", this->options.outputQueueSize);
        }

        void subscribe() override {
            switch (this->options.subscriptionType) {
                case MessagePassingType::UNIQUE_PTR:
                {
                    this->inputSubscriber = create_subscription<T>(
                        "input", this->options.inputQueueSize,
                        [this](typename T::UniquePtr msg) {
                            FilterChainBase<T>::callbackUnique(std::move(msg));
                        }
                    );
                    break;
                }
                case MessagePassingType::SHARED_PTR:
                {
                    this->inputSubscriber = create_subscription<T>(
                        "input", this->options.inputQueueSize,
                        [this](const typename T::ConstSharedPtr& msg) {
                            FilterChainBase<T>::callbackShared(msg);
                        }
                    );
                    break;
                }
                case MessagePassingType::REFERENCE:
                {
                    this->inputSubscriber = create_subscription<T>(
                        "input", this->options.inputQueueSize,
                        [this](const T& msg) {
                            FilterChainBase<T>::callbackReference(msg);
                        });
                    break;
                }
                default:
                    assert(false && "Unexpected subscription type");
            }
        }

        void publishUnique(typename T::UniquePtr msg) override {
            if (!this->is_activated())
                return;

            this->outputPublisher->publish(std::move(msg));
        }

        void publishShared(const typename T::ConstSharedPtr&) override {
            throw std::runtime_error("LifeCycleFilterChainNode does not support shared_ptr publications");
        }

        void publishReference(const T& msg) override {
            if (!this->is_activated())
                return;

            this->outputPublisher->publish(msg);
        }

    private:
        std::shared_ptr<rclcpp::Subscription<T>> inputSubscriber;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<T>> outputPublisher;
    };

}
