// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_filters/FilterChainNodelet.h>
#include <sensor_filters/PointCloud2FilterChainBase.h>

DECLARE_SENSOR_FILTER_BASE(PointCloud2, PointCloud2FilterChainBase, "cloud")  // NOLINT(cert-err58-cpp)
