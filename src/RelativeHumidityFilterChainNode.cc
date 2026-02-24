// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_filters/FilterChainNode.h>
#include <sensor_msgs/msg/relative_humidity.hpp>

int main(const int argc, char** argv) {
    sensor_filters::spinFilterChain<sensor_msgs::msg::RelativeHumidity>("humidity_filter_chain", argc, argv);
}
