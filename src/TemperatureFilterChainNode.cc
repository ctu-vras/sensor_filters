// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_msgs/Temperature.h>

#include <sensor_filters/FilterChainNode.h>

int main(int argc, char** argv)
{
  sensor_filters::spinFilterChain<sensor_msgs::Temperature>("temperature_filter_chain", argc, argv);
}
