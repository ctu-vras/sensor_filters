/**
 * \file
 * \brief An example how you can write a custom nodelet that runs the sensor filter chain and does some other tasks.
 * \author Martin Pecka
 * SPDX-License-Identifier: Unlicense
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_filters/FilterChainNodelet.h>

class MyNodelet : public sensor_filters::FilterChainNodelet<sensor_msgs::PointCloud2>
{
  protected: void onInit() override
  {
    sensor_filters::FilterChainNodelet<sensor_msgs::PointCloud2>::onInit();
    // custom nodelet code
  }

  protected: bool filter(const sensor_msgs::PointCloud2& msgIn, sensor_msgs::PointCloud2& msgOut) override
  {
    // your custom logic that is run before filtering each message
    return sensor_filters::FilterChainNodelet<sensor_msgs::PointCloud2>::filter(msgIn, msgOut);
  }
};

PLUGINLIB_EXPORT_CLASS(MyNodelet, nodelet::Nodelet)