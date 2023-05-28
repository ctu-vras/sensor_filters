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

#include <point_cloud_transport/point_cloud_transport.h>
#include <point_cloud_transport/publisher.h>
#include <point_cloud_transport/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_filters/FilterChainBase.h>

namespace sensor_filters
{
class PointCloud2FilterChainBase : public FilterChainBase<sensor_msgs::PointCloud2>
{
  public: PointCloud2FilterChainBase() : FilterChainBase<sensor_msgs::PointCloud2>() {}

protected:
    std::unique_ptr<point_cloud_transport::PointCloudTransport> pct;
    point_cloud_transport::Publisher pctPublisher;
    point_cloud_transport::Subscriber pctSubscriber;

    void initFilters(const std::string &filterChainNamespace, ros::NodeHandle filterNodeHandle,
                     ros::NodeHandle topicNodeHandle, const bool useSharedPtrMessages, const size_t inputQueueSize,
                     const size_t outputQueueSize) override;

    void advertise() override;

    void subscribe() override;

    void publishShared(const sensor_msgs::PointCloud2ConstPtr& msg) override;
};

}
