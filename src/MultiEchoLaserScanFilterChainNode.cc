// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_filters/FilterChainNode.h>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>

int main(const int argc, char** argv) {
    sensor_filters::spinFilterChain<sensor_msgs::msg::MultiEchoLaserScan>("scan_filter_chain", argc, argv);
}
