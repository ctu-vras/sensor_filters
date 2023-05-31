// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>

#include <sensor_filters/FilterChainNodelet.h>
#include <sensor_filters/ImageFilterChainBase.h>

DECLARE_SENSOR_FILTER_BASE(Image, ImageFilterChainBase, "image")  // NOLINT(cert-err58-cpp)
