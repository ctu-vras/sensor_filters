// SPDX-License-Identifier: Unlicense
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief An example how you can write a custom node that runs the sensor filter chain and does some other tasks.
 */

#include <sensor_msgs/PointCloud2.h>
#include <sensor_filters/FilterChainNode.h>

class MyClass : public sensor_filters::FilterChainNode<sensor_msgs::PointCloud2>
{
public:
  explicit MyClass(ros::NodeHandle nh) :
    sensor_filters::FilterChainNode<sensor_msgs::PointCloud2>("my_filter_chain", nh, nh)
  {
    // Constructor of your class
  }

protected:
  bool filter(const sensor_msgs::PointCloud2& msgIn, sensor_msgs::PointCloud2& msgOut) override
  {
    // your custom logic that is run before filtering each message
    return sensor_filters::FilterChainNode<sensor_msgs::PointCloud2>::filter(msgIn, msgOut);
  }

  // custom class code
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_filter_chain");
  ros::NodeHandle nh("~");
  MyClass node(nh);
  ros::spin();
}
