// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief A macro that registers a generic filter with all sensor message types.
 */

#pragma once

#include <boost/preprocessor/seq/for_each.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

// Keep these lists in sync with those in cmake/UniversalFilter.cmake

//! \brief All "top-level" message types from sensor_msgs that contain a header.
#define SENSOR_MSGS_SEQ \
    (sensor_msgs::msg::BatteryState) \
    (sensor_msgs::msg::CameraInfo) \
    (sensor_msgs::msg::CompressedImage) \
    (sensor_msgs::msg::FluidPressure) \
    (sensor_msgs::msg::Illuminance) \
    (sensor_msgs::msg::Image) \
    (sensor_msgs::msg::Imu) \
    (sensor_msgs::msg::JointState) \
    (sensor_msgs::msg::Joy) \
    (sensor_msgs::msg::LaserScan) \
    (sensor_msgs::msg::MagneticField) \
    (sensor_msgs::msg::MultiDOFJointState) \
    (sensor_msgs::msg::MultiEchoLaserScan) \
    (sensor_msgs::msg::NavSatFix) \
    (sensor_msgs::msg::PointCloud) \
    (sensor_msgs::msg::PointCloud2) \
    (sensor_msgs::msg::Range) \
    (sensor_msgs::msg::RelativeHumidity) \
    (sensor_msgs::msg::Temperature) \
    (sensor_msgs::msg::TimeReference)

//! \brief All "top-level" message types from sensor_msgs that do not contain a header.
#define SENSOR_MSGS_NO_HEADER_SEQ \
    (sensor_msgs::msg::JoyFeedbackArray)

#define REGISTER_TEMPLATED_FILTER_FOR_MSG(filter, msg) \
    PLUGINLIB_EXPORT_CLASS(filter<msg>, filters::FilterBase<msg>)

#define REGISTER_TEMPLATED_FILTER_FOR_MSG_BOOST_PP_SEQ_HELPER(r, filter, msg) \
    REGISTER_TEMPLATED_FILTER_FOR_MSG(filter, msg)

#define REGISTER_TEMPLATED_FILTER_FOR_MSGS(filter, msg_seq) \
    BOOST_PP_SEQ_FOR_EACH(REGISTER_TEMPLATED_FILTER_FOR_MSG_BOOST_PP_SEQ_HELPER, filter, msg_seq)

//! \brief Register the given templated filters::FilterBase implementation for all sensor_msgs messages with header.
//! \note Don't forget to register the filters by calling register_universal_filter_description_file() CMake macro.
#define REGISTER_UNIVERSAL_SENSOR_MSGS_FILTER(filter) \
    REGISTER_TEMPLATED_FILTER_FOR_MSGS(filter, SENSOR_MSGS_SEQ)

//! \brief Register the given templated filters::FilterBase implementation for all sensor_msgs messages without header.
//! \note Don't forget to register the filters by calling register_universal_filter_description_file() CMake macro.
#define REGISTER_NO_HEADER_SENSOR_MSGS_FILTER(filter) \
    REGISTER_TEMPLATED_FILTER_FOR_MSGS(filter, SENSOR_MSGS_NO_HEADER_SEQ)
