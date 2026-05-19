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
    template <typename T, typename Base = FilterChainNodeBase<T>>
    class FilterChainNode : public rclcpp::Node {
    public:
        // TODO 2026-03-16 (solonovamax): support parameter callback
        explicit FilterChainNode(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
            const FilterChainOptions& defaultChainOptions = Base::DEFAULT_CHAIN_OPTIONS) :
            rclcpp::Node(name, options), filterChain(std::make_unique<Base>(*this, name, defaultChainOptions))
        {
            this->filterChain->on_configure();
            this->filterChain->on_activate();
        }

        ~FilterChainNode() override
        {
            this->filterChain->on_deactivate();
            this->filterChain->on_shutdown();
            this->filterChain.reset();
        }

    protected:
        std::unique_ptr<FilterChainBase<T>> filterChain;
    };

    template <typename T, typename Base = FilterChainNodeBase<T>>
    class LifecycleFilterChainNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        // TODO 2026-03-16 (solonovamax): support parameter callback
        explicit LifecycleFilterChainNode(const std::string& name,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
            const FilterChainOptions& defaultChainOptions = Base::DEFAULT_CHAIN_OPTIONS) :
            LifecycleNode(name, options), filterChain(std::make_unique<Base>(*this, name, defaultChainOptions))
        {
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
            try {
                this->filterChain->on_configure();
            } catch (const std::runtime_error&) {
                return CallbackReturn::ERROR;
            }

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override
        {
            this->filterChain->on_activate();
            return rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override
        {
            this->filterChain->on_deactivate();
            return rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        }

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override {
            // reset state to before on_configure()
            this->filterChain->on_cleanup();
            return rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override {
            this->filterChain->on_shutdown();
            return rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
        }

        CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override {
            // currently an error can only occur in on_configure, so we don't need to check the previous state
            // if other transitions are ever changed so that they can error, then this needs to be updated.
            this->filterChain->on_cleanup();
            return rclcpp_lifecycle::LifecycleNode::on_error(previous_state);
        }

    protected:
        std::unique_ptr<FilterChainBase<T>> filterChain;
    };

}
