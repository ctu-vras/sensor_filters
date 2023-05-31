// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/NavSatFix.h>

#include <sensor_filters/FilterChainNodelet.h>

DECLARE_SENSOR_FILTER(NavSatFix, "nav_sat_fix")  // NOLINT(cert-err58-cpp)
