// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <filters/filter_base.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sensor_filters {

template<typename T>
class ChangeHeader : public filters::FilterBase<T> {
protected:
  bool configure() override {
    {
      std::string frame_id_param;
      if (this->getParam("frame_id_prefix", frame_id_param) && !frame_id_param.empty()) {
        new_frame_id_prefix_ = frame_id_param;
      }

      if (this->getParam("frame_id_suffix", frame_id_param) && !frame_id_param.empty()) {
        new_frame_id_suffix_ = frame_id_param;
      }

      if (this->getParam("frame_id", frame_id_param, true, "?") && frame_id_param != "?") {
        new_frame_id_ = frame_id_param;
      }
    }

    {
      const auto kNan = std::numeric_limits<double>::quiet_NaN();
      double stampParam {kNan};
      if (this->getParam("stamp_relative", stampParam, true, kNan) && std::isfinite(stampParam)) {
        new_stamp_rel_ = rclcpp::Duration::from_seconds(stampParam);
      }

      if (this->getParam("stamp", stampParam, true, kNan) && std::isfinite(stampParam)) {
        const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(stampParam));
        new_stamp_abs_ = rclcpp::Time(nanos.count());
      }
    }

    return true;
  }

public:
  bool update(const T& data_in, T& data_out) override {
    data_out = data_in;

    if (new_frame_id_prefix_.has_value()) {
      data_out.header.frame_id = new_frame_id_prefix_.value() + data_out.header.frame_id;
    }

    if (new_frame_id_suffix_.has_value()) {
      data_out.header.frame_id += new_frame_id_suffix_.value();
    }

    if (new_frame_id_.has_value()) {
      data_out.header.frame_id = new_frame_id_.value();
    }

    if (new_stamp_rel_.has_value()) {
      data_out.header.stamp = rclcpp::Time(data_out.header.stamp) + new_stamp_rel_.value();
    }

    if (new_stamp_abs_.has_value()) {
      data_out.header.stamp = new_stamp_abs_.value();
    }

    return true;
  }

private:
  std::optional<std::string> new_frame_id_;
  std::optional<std::string> new_frame_id_prefix_;
  std::optional<std::string> new_frame_id_suffix_;

  std::optional<rclcpp::Time> new_stamp_abs_;
  std::optional<rclcpp::Duration> new_stamp_rel_;
};

} // namespace sensor_filters

#include <sensor_filters/RegisterFilter.hpp>
REGISTER_UNIVERSAL_SENSOR_MSGS_FILTER(sensor_filters::ChangeHeader)
