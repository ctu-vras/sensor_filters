// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_filters/FilterChainNode.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

int main(const int argc, char** argv) {
    sensor_filters::spinFilterChain<sensor_msgs::msg::NavSatFix>("nav_sat_fix_filter_chain", argc, argv);
}
