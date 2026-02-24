// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_filters/FilterChainNode.hpp>
#include <sensor_filters/ImageFilterChainBase.hpp>
#include <sensor_msgs/msg/image.hpp>

int main(const int argc, char** argv) {
    sensor_filters::spinFilterChain<sensor_msgs::msg::Image, sensor_filters::ImageFilterChainBase>(
        "image_filter_chain", argc, argv);
}
