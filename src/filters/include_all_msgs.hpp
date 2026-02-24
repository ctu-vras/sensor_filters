// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief A macro that registers a generic filter with all sensor message types.
 */

#pragma once

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>

#define REGISTER_ALL_MSG_FILTER(filter) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::CompressedImage>, filters::FilterBase<sensor_msgs::CompressedImage>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::Image>, filters::FilterBase<sensor_msgs::Image>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::Imu>, filters::FilterBase<sensor_msgs::Imu>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::Joy>, filters::FilterBase<sensor_msgs::Joy>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::LaserScan>, filters::FilterBase<sensor_msgs::LaserScan>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::MagneticField>, filters::FilterBase<sensor_msgs::MagneticField>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::MultiEchoLaserScan>, filters::FilterBase<sensor_msgs::MultiEchoLaserScan>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::NavSatFix>, filters::FilterBase<sensor_msgs::NavSatFix>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::PointCloud>, filters::FilterBase<sensor_msgs::PointCloud>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::PointCloud2>, filters::FilterBase<sensor_msgs::PointCloud2>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::Range>, filters::FilterBase<sensor_msgs::Range>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::RelativeHumidity>, filters::FilterBase<sensor_msgs::RelativeHumidity>) \
PLUGINLIB_EXPORT_CLASS(filter<sensor_msgs::Temperature>, filters::FilterBase<sensor_msgs::Temperature>)
