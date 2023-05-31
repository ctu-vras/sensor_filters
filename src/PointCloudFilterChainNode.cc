// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_msgs/PointCloud.h>

#include <sensor_filters/FilterChainNode.h>

int main(int argc, char** argv)
{
  sensor_filters::spinFilterChain<sensor_msgs::PointCloud>("cloud_filter_chain", argc, argv);
}
