// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for a sensor filter chain node.
 */

#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_filters/FilterChainBase.hpp>

namespace sensor_filters {
    template <typename T, typename Base = FilterChainNodeBase<T>>
    class FilterChainNode : public rclcpp::Node {
    public:
        explicit FilterChainNode(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
            const FilterChainOptions& default_chain_options = Base::kDefaultChainOptions) :
            rclcpp::Node(name, options), filter_chain_base_(std::make_unique<Base>(*this, name, default_chain_options))
        {
            filter_chain_base_->on_configure();
            filter_chain_base_->on_activate();
        }

        ~FilterChainNode() override
        {
            filter_chain_base_->on_deactivate();
            filter_chain_base_->on_shutdown();
            filter_chain_base_.reset();
        }

    protected:
        std::unique_ptr<FilterChainBase<T>> filter_chain_base_;
    };

    template <typename T, typename Base = FilterChainNodeBase<T>>
    class LifecycleFilterChainNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit LifecycleFilterChainNode(const std::string& name,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
            const FilterChainOptions& default_chain_options = Base::kDefaultChainOptions) :
            LifecycleNode(name, options), filter_chain_base_(std::make_unique<Base>(*this, name, default_chain_options))
        {
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
            try {
                filter_chain_base_->on_configure();
            } catch (const std::runtime_error&) {
                return CallbackReturn::ERROR;
            }

            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override
        {
            filter_chain_base_->on_activate();
            return rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override
        {
            filter_chain_base_->on_deactivate();
            return rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        }

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override {
            // reset state to before on_configure()
            filter_chain_base_->on_cleanup();
            return rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override {
            filter_chain_base_->on_shutdown();
            return rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
        }

        CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override {
            // currently an error can only occur in on_configure, so we don't need to check the previous state
            // if other transitions are ever changed so that they can error, then this needs to be updated.
            filter_chain_base_->on_cleanup();
            return rclcpp_lifecycle::LifecycleNode::on_error(previous_state);
        }

    protected:
        std::unique_ptr<FilterChainBase<T>> filter_chain_base_;
    };

}
