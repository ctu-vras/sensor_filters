// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <chrono>
#include <optional>
#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>

#include "include_all_msgs.hpp"

namespace sensor_filters {
    template <typename T>
    class ChangeHeader : public filters::FilterBase<T> {
    protected:
        bool configure() override {
            {
                std::string frameIdParam;
                if (this->getParam("frame_id_prefix", frameIdParam) && !frameIdParam.empty())
                    this->newFrameIdPrefix = frameIdParam;

                if (this->getParam("frame_id_suffix", frameIdParam) && !frameIdParam.empty())
                    this->newFrameIdSuffix = frameIdParam;

                if (this->getParam("frame_id", frameIdParam, true, "?") && frameIdParam != "?")
                    this->newFrameId = frameIdParam;
            }

            {
                const auto nan = std::numeric_limits<double>::quiet_NaN();
                double stampParam {nan};
                if (this->getParam("stamp_relative", stampParam, true, nan) && std::isfinite(stampParam))
                    this->newStampRel = rclcpp::Duration::from_seconds(stampParam);

                if (this->getParam("stamp", stampParam, true, nan) && std::isfinite(stampParam)) {
                    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(stampParam));
                    this->newStampAbs = rclcpp::Time(nanos.count());
                }
            }

            return true;
        }

    public:
        bool update(const T& data_in, T& data_out) override {
            data_out = data_in;

            if (this->newFrameIdPrefix.has_value())
                data_out.header.frame_id = this->newFrameIdPrefix.value() + data_out.header.frame_id;

            if (this->newFrameIdSuffix.has_value())
                data_out.header.frame_id += this->newFrameIdSuffix.value();

            if (this->newFrameId.has_value())
                data_out.header.frame_id = this->newFrameId.value();

            if (this->newStampRel.has_value())
                data_out.header.stamp = rclcpp::Time(data_out.header.stamp) + this->newStampRel.value();

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

        std::optional<rclcpp::Time> newStampAbs;
        std::optional<rclcpp::Duration> newStampRel;
    };
}

REGISTER_ALL_MSG_FILTERS(sensor_filters::ChangeHeader)
