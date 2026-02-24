// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_msgs/msg/joy.hpp>

#include <sensor_filters/FilterChainNode.h>

int main(const int argc, char** argv) {
  sensor_filters::spinFilterChain<sensor_msgs::msg::Joy>("joy_filter_chain", argc, argv);
}
