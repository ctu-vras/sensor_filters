// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <optional>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <pluginlib/class_list_macros.h>

#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_chain.hpp>
#else
#include <filters/filter_chain.h>
#endif

#include "include_all_msgs.hh"

namespace sensor_filters
{

template<typename T>
class ChangeHeader : public filters::FilterBase<T>
{
protected:
  bool configure() override
  {
    {
      std::string frameIdParam;
      if (this->getParam("frame_id_prefix", frameIdParam))
        this->newFrameIdPrefix = frameIdParam;

      if (this->getParam("frame_id_suffix", frameIdParam))
        this->newFrameIdSuffix = frameIdParam;

      if (this->getParam("frame_id", frameIdParam))
        this->newFrameId = frameIdParam;
    }

    {
      uint32_t seqParam;
      if (this->getParam("seq_relative", seqParam))
        this->newSeqRel = seqParam;

      if (this->getParam("seq", seqParam))
        this->newSeqAbs = seqParam;
    }

    {
      double stampParam;
      if (this->getParam("stamp_relative", stampParam))
        this->newStampRel = ros::Duration(stampParam);

      if (this->getParam("stamp", stampParam))
        this->newStampAbs = ros::Time(stampParam);
    }

    return true;
  }

public:
  bool update(const T& data_in, T& data_out) override
  {
    data_out = data_in;

    if (this->newFrameIdPrefix.has_value())
      data_out.header.frame_id = this->newFrameIdPrefix.value() + data_out.header.frame_id;

    if (this->newFrameIdSuffix.has_value())
      data_out.header.frame_id += this->newFrameIdSuffix.value();

    if (this->newFrameId.has_value())
      data_out.header.frame_id = this->newFrameId.value();

    if (this->newSeqRel.has_value())
      data_out.header.seq += this->newSeqRel.value();

    if (this->newSeqAbs.has_value())
      data_out.header.seq = this->newSeqAbs.value();

    if (this->newStampRel.has_value())
      data_out.header.stamp += this->newStampRel.value();

    if (this->newStampAbs.has_value())
      data_out.header.stamp = this->newStampAbs.value();

    return true;
  }

private:
  std::optional<std::string> newFrameId;
  std::optional<std::string> newFrameIdPrefix;
  std::optional<std::string> newFrameIdSuffix;

  std::optional<uint32_t> newSeqAbs;
  std::optional<uint32_t> newSeqRel;

  std::optional<ros::Time> newStampAbs;
  std::optional<ros::Duration> newStampRel;
};

}

REGISTER_ALL_MSG_FILTER(sensor_filters::ChangeHeader)
