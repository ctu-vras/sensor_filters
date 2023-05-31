// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <sensor_msgs/Image.h>

#include <sensor_filters/FilterChainNode.h>
#include <sensor_filters/ImageFilterChainBase.h>

int main(int argc, char** argv)
{
  sensor_filters::spinFilterChain<sensor_msgs::Image, sensor_filters::ImageFilterChainBase>(
    "image_filter_chain", argc, argv);
}
