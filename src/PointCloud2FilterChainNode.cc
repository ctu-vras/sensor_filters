// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_msgs/PointCloud2.h>

#include <sensor_filters/FilterChainNode.h>
#include <sensor_filters/PointCloud2FilterChainBase.h>

int main(int argc, char** argv)
{
  sensor_filters::spinFilterChain<sensor_msgs::PointCloud2, sensor_filters::PointCloud2FilterChainBase>(
    "cloud_filter_chain", argc, argv);
}
