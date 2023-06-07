// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#pragma once

/**
 * \file
 * \brief Base for all sensor filter chains.
 */

#include <string>

#include <ros/ros.h>

#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_chain.hpp>
#else

#include <filters/filter_chain.h>

#endif


namespace sensor_filters
{

template<typename T>
class FilterChainBase
{
protected:
  ros::Subscriber inputSubscriber;
  ros::Publisher outputPublisher;
  ros::NodeHandle topicNodeHandle;
  size_t inputQueueSize{10u};
  size_t outputQueueSize{10u};
  bool useSharedPtrMessages{true};

  filters::FilterChain<T> filterChain;
  T msg;

  typedef ros::message_traits::DataType<T> DataType;

public:
  FilterChainBase() :
    filterChain(std::string(DataType::value()).replace(std::string(DataType::value()).find('/'), 1, "::"))
  {
  }

  virtual ~FilterChainBase() = default;

protected:
  virtual void initFilters(
    const std::string& filterChainNamespace, ros::NodeHandle filterNodeHandle, ros::NodeHandle topicNodeHandle,
    const bool useSharedPtrMessages, const size_t inputQueueSize, const size_t outputQueueSize)
  {
    if (!this->filterChain.configure(filterChainNamespace, filterNodeHandle))
    {
      ROS_ERROR_STREAM("Configuration of filter chain for "
                         << DataType::value() << " is invalid, the chain will not be run.");
      throw std::runtime_error("Filter configuration error");
    }

    ROS_INFO_STREAM("Configured filter chain of type " << DataType::value() << " from namespace "
                                                       << filterNodeHandle.getNamespace() << "/"
                                                       << filterChainNamespace);

    this->topicNodeHandle = topicNodeHandle;
    this->outputQueueSize = outputQueueSize;
    this->inputQueueSize = inputQueueSize;
    this->useSharedPtrMessages = useSharedPtrMessages;

    this->advertise();
    this->subscribe();
  }

  virtual void advertise()
  {
    this->outputPublisher = this->topicNodeHandle.template advertise<T>("output", this->outputQueueSize);
  }

  virtual void subscribe()
  {
    if (this->useSharedPtrMessages)
      this->inputSubscriber = this->topicNodeHandle.subscribe(
        "input", this->inputQueueSize, &FilterChainBase::callbackShared, this);
    else
      this->inputSubscriber = this->topicNodeHandle.subscribe(
        "input", this->inputQueueSize, &FilterChainBase::callbackReference, this);
  }

  virtual void publishShared(const typename T::ConstPtr& msg)
  {
    this->outputPublisher.publish(msg);
  }

  virtual void publishReference(const T& msg)
  {
    this->outputPublisher.publish(msg);
  }

  virtual void callbackShared(const typename T::ConstPtr& msgIn)
  {
    typename T::Ptr msgOut(new T);
    if (this->filter(*msgIn, *msgOut))
      this->publishShared(msgOut);
  }

  virtual void callbackReference(const T& msgIn)
  {
    if (this->filter(msgIn, this->msg))
      this->publishReference(this->msg);
  }

  virtual bool filter(const T& msgIn, T& msgOut)
  {
    ros::WallTime start = ros::WallTime::now();
    if (!this->filterChain.update(msgIn, msgOut))
    {
      ROS_ERROR_THROTTLE(1, "Filtering data from time %i.%i failed.",
                         msgIn.header.stamp.sec, msgIn.header.stamp.nsec);
      return false;
    }
    ROS_DEBUG_STREAM("Filtering took " << (ros::WallTime::now() - start).toSec() << " s.");
    return true;
  }
};

}
