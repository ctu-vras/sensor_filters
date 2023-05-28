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
#include <memory>
#include <string>

#include <image_transport/image_transport.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>

#include <sensor_filters/FilterChainBase.h>
#include <sensor_filters/ImageFilterChainBase.h>

namespace sensor_filters
{

void ImageFilterChainBase::initFilters(const std::string &filterChainNamespace, ros::NodeHandle filterNodeHandle,
                                       ros::NodeHandle topicNodeHandle, const bool useSharedPtrMessages,
                                       const size_t inputQueueSize, const size_t outputQueueSize)
{
  this->it = std::make_unique<image_transport::ImageTransport>(topicNodeHandle);
  FilterChainBase::initFilters(filterChainNamespace, filterNodeHandle, topicNodeHandle, useSharedPtrMessages,
                               inputQueueSize, outputQueueSize);
}

void ImageFilterChainBase::advertise() {
  const auto resolvedOutput = this->topicNodeHandle.resolveName("output");
  this->itPublisher = this->it->advertise(resolvedOutput, this->outputQueueSize);
}

void ImageFilterChainBase::subscribe() {
  const auto resolvedInput = this->topicNodeHandle.resolveName("input");
  this->itSubscriber = this->it->subscribe<ImageFilterChainBase>(
      resolvedInput, this->inputQueueSize, &ImageFilterChainBase::callbackShared, this);
}

void ImageFilterChainBase::publishShared(const sensor_msgs::ImageConstPtr &msg) {
  this->itPublisher.publish(msg);
}

}
