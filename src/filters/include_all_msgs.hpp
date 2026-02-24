// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief A macro that registers a generic filter with all sensor message types.
 */

#pragma once

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

#define REGISTER_ALL_MSG_FILTER(filter) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::CompressedImage>, filters::FilterBase<sensor_msgs::msg::CompressedImage>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::Image>, filters::FilterBase<sensor_msgs::msg::Image>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::Imu>, filters::FilterBase<sensor_msgs::msg::Imu>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::Joy>, filters::FilterBase<sensor_msgs::msg::Joy>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::LaserScan>, filters::FilterBase<sensor_msgs::msg::LaserScan>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::MagneticField>, filters::FilterBase<sensor_msgs::msg::MagneticField>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::MultiEchoLaserScan>, filters::FilterBase<sensor_msgs::msg::MultiEchoLaserScan>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::NavSatFix>, filters::FilterBase<sensor_msgs::msg::NavSatFix>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::PointCloud>, filters::FilterBase<sensor_msgs::msg::PointCloud>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::PointCloud2>, filters::FilterBase<sensor_msgs::msg::PointCloud2>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::Range>, filters::FilterBase<sensor_msgs::msg::Range>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::RelativeHumidity>, filters::FilterBase<sensor_msgs::msg::RelativeHumidity>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::msg::Temperature>, filters::FilterBase<sensor_msgs::msg::Temperature>)
