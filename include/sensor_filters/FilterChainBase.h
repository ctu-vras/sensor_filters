#pragma once
/*
 * Copyright (c) 2021, Czech Technical University in Prague
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_chain.hpp>
#else
#include <filters/filter_chain.h>
#endif


namespace sensor_filters
{

template <typename T>
class FilterChainBase
{
  protected: ros::Subscriber inputSubscriber;
  protected: ros::Publisher outputPublisher;

  protected: filters::FilterChain<T> filterChain;
  protected: T msg;

  protected: typedef ros::message_traits::DataType<T> DataType;

  public: FilterChainBase() :
    filterChain(std::string(DataType::value()).replace(std::string(DataType::value()).find('/'), 1, "::"))
  {
  }

  public: virtual ~FilterChainBase() = default;

  protected: virtual void initFilters(
    const std::string& filterChainNamespace, ros::NodeHandle filterNodeHandle, ros::NodeHandle topicNodeHandle,
    const bool useSharedPtrMessages, const size_t inputQueueSize, const size_t outputQueueSize)
  {
    if (!this->filterChain.configure(filterChainNamespace, filterNodeHandle))
    {
      ROS_ERROR_STREAM("Configuration of filter chain for "
        << DataType::value() << " is invalid, the chain will not be run.");
      throw std::runtime_error("Filter configuration error");
    }

    ROS_INFO_STREAM("Configured filter chain of type " << DataType::value() <<
      " from namespace " << filterNodeHandle.getNamespace() << "/" << filterChainNamespace);

    this->outputPublisher = topicNodeHandle.template advertise<T>("output", outputQueueSize);
    if (useSharedPtrMessages)
      this->inputSubscriber = topicNodeHandle.subscribe("input", inputQueueSize, &FilterChainBase::callbackShared, this);
    else
      this->inputSubscriber = topicNodeHandle.subscribe("input", inputQueueSize, &FilterChainBase::callbackReference, this);
  }

  protected: virtual void callbackShared(const typename T::ConstPtr& msgIn)
  {
    typename T::Ptr msgOut(new T);
    if (this->filter(*msgIn, *msgOut))
      this->outputPublisher.publish(msgOut);
  }

  protected: virtual void callbackReference(const T& msgIn)
  {
    if (this->filter(msgIn, this->msg))
      this->outputPublisher.publish(this->msg);
  }

  protected: virtual bool filter(const T& msgIn, T& msgOut)
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