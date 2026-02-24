// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for a sensor filter chain node.
 */

#include <string>
#include <sensor_filters/FilterChainBase.hpp>

namespace sensor_filters {
    template <typename T, typename Base = FilterChainBase<T>>
    class FilterChainNode : public rclcpp::Node, public Base {
    public:
        explicit FilterChainNode(const std::string& name, const rclcpp::NodeOptions& options) : Node(name, options), Base() {
            this->initFilters(
                name, this->node, false,
                this->declare_parameter("input_queue_size", 10),
                this->declare_parameter("output_queue_size", 10)
            );
        }

        //! use a composable node instead
        [[deprecated]] explicit FilterChainNode(const std::string& name) : Node(name), Base() {
            this->initFilters(
                name, this->node, false,
                this->declare_parameter("input_queue_size", 10),
                this->declare_parameter("output_queue_size", 10)
            );
        }
    };

    //! this method is deprecated, and instead a composable node should be used
    template <typename T, typename Base = FilterChainBase<T>>
    [[deprecated]] void spinFilterChain(const std::string& name, const int argc, char** argv) {
        rclcpp::init(argc, argv);
        const auto node = std::make_shared<FilterChainNode<T, Base>>(name);
        rclcpp::spin(node);
    }
}
