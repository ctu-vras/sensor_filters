// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/MagneticField.h>

#include <sensor_filters/FilterChainNodelet.h>

DECLARE_SENSOR_FILTER(MagneticField, "magnetic_field")  // NOLINT(cert-err58-cpp)
