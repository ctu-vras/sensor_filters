// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief A macro that registers a generic filter with all sensor message types.
 */

#pragma once

#include <boost/preprocessor/seq/for_each.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#define SENSOR_MSGS_SEQ \
    (sensor_msgs::msg::CompressedImage) \
    (sensor_msgs::msg::Image) \
    (sensor_msgs::msg::Imu) \
    (sensor_msgs::msg::Joy) \
    (sensor_msgs::msg::LaserScan) \
    (sensor_msgs::msg::MagneticField) \
    (sensor_msgs::msg::MultiEchoLaserScan) \
    (sensor_msgs::msg::NavSatFix) \
    (sensor_msgs::msg::PointCloud) \
    (sensor_msgs::msg::PointCloud2) \
    (sensor_msgs::msg::Range) \
    (sensor_msgs::msg::RelativeHumidity) \
    (sensor_msgs::msg::Temperature)

#define REGISTER_MSG_FILTER(r, filter, msg) \
    PLUGINLIB_EXPORT_CLASS(filter<msg>, filters::FilterBase<msg>)

#define REGISTER_MSG_FILTERS(filter, msg_seq) \
    BOOST_PP_SEQ_FOR_EACH(REGISTER_MSG_FILTER, filter, msg_seq)

#define REGISTER_ALL_MSG_FILTERS(filter) \
    REGISTER_MSG_FILTERS(filter, SENSOR_MSGS_SEQ)
