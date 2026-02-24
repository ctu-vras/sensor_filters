// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_filters/FilterChainNode.h>
#include <sensor_filters/PointCloud2FilterChainBase.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(const int argc, char** argv) {
    sensor_filters::spinFilterChain<sensor_msgs::msg::PointCloud2, sensor_filters::PointCloud2FilterChainBase>("cloud_filter_chain", argc, argv);
}
