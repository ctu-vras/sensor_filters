// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_filters/FilterChainNode.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

int main(const int argc, char** argv) {
    sensor_filters::spinFilterChain<sensor_msgs::msg::MagneticField>("magnetic_field_filter_chain", argc, argv);
}
