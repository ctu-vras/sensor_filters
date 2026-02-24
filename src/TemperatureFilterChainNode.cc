// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_msgs/msg/temperature.hpp>

#include <sensor_filters/FilterChainNode.h>

int main(const int argc, char** argv) {
  sensor_filters::spinFilterChain<sensor_msgs::msg::Temperature>("temperature_filter_chain", argc, argv);
}
