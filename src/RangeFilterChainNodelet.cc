// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Range.h>

#include <sensor_filters/FilterChainNodelet.h>

DECLARE_SENSOR_FILTER(Range, "range")  // NOLINT(cert-err58-cpp)
