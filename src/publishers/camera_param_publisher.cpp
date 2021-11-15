/*
 * Copyright (c) 2021 Roboception GmbH
 * All rights reserved
 *
 * Author: Heiko Hirschmueller
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "camera_param_publisher.hpp"

#include <rc_genicam_api/pixel_formats.h>
#include <rc_genicam_api/config.h>

namespace rc
{
CameraParamPublisher::CameraParamPublisher(
  rclcpp::Node * node, const std::string & frame_id,
  bool left)
: GenICam2RosPublisher(frame_id)
{
  // advertise topic

  if (left) {
    pub = node->create_publisher<rc_common_msgs::msg::CameraParam>("stereo/left/camera_param", 1);
  } else {
    pub = node->create_publisher<rc_common_msgs::msg::CameraParam>("stereo/right/camera_param", 1);
  }
}

bool CameraParamPublisher::used()
{
  return pub->get_subscription_count() > 0;
}

void CameraParamPublisher::requiresComponents(int & components, bool &)
{
  if (pub->get_subscription_count() > 0) {
    components |= ComponentIntensity;
  }
}

namespace
{

template<class T>
inline rc_common_msgs::msg::KeyValue getKeyValue(const char * key, T value)
{
  rc_common_msgs::msg::KeyValue ret;

  ret.key = key;
  ret.value = std::to_string(value);

  return ret;
}

inline rc_common_msgs::msg::KeyValue getKeyValueString(const char * key, const std::string &value)
{
  rc_common_msgs::msg::KeyValue ret;

  ret.key=key;
  ret.value=value;

  return ret;
}

}

void CameraParamPublisher::publish(const rcg::Buffer * buffer, uint32_t, uint64_t pixelformat)
{
  if (nodemap && pub->get_subscription_count() > 0 &&
    (pixelformat == Mono8 || pixelformat == YCbCr411_8 || pixelformat == RGB8))
  {
    uint64_t time = buffer->getTimestampNS();

    rc_common_msgs::msg::CameraParam param;

    param.header.frame_id = frame_id;
    param.header.stamp.sec = time / 1000000000ul;
    param.header.stamp.nanosec = time % 1000000000ul;

    // get list of all available IO lines and iterate over them
    // to get input/output masks and LineSource values for outputs

    std::vector<std::string> lines;
    rcg::getEnum(nodemap, "LineSelector", lines, true);
    uint32_t input_mask = 0;
    uint32_t output_mask = 0;
    for (size_t i = 0; i < lines.size(); i++)
    {
      rcg::setEnum(nodemap, "LineSelector", lines[i].c_str(), true);
      std::string io = rcg::getEnum(nodemap, "LineMode");

      if (io == "Input")
      {
        input_mask |= 1 << i;
      }

      if (io == "Output")
      {
        output_mask |= 1 << i;

        // for output also get LineSource for this image
        rcg::setEnum(nodemap, "ChunkLineSelector", lines[i].c_str(), true);
        rc_common_msgs::msg::KeyValue line_source;
        line_source.key = lines[i];
        line_source.value = rcg::getEnum(nodemap, "ChunkLineSource", true);
        param.line_source.push_back(line_source);
      }
    }
    param.line_status_all = rcg::getInteger(nodemap, "ChunkLineStatusAll", 0, 0, true);

    param.gain = rcg::getFloat(nodemap, "ChunkGain", 0, 0, true);
    param.exposure_time = rcg::getFloat(nodemap, "ChunkExposureTime", 0, 0, true) / 1000000l;

    param.extra_data.push_back(getKeyValue("noise", rcg::getFloat(nodemap, "ChunkRcNoise", 0, 0, false)));
    param.extra_data.push_back(getKeyValue("test", rcg::getBoolean(nodemap, "ChunkRcTest", false)));
    param.extra_data.push_back(getKeyValue("brightness", rcg::getFloat(nodemap, "ChunkRcBrightness", 0, 0, false)));
    param.extra_data.push_back(getKeyValue("input_mask", input_mask));
    param.extra_data.push_back(getKeyValue("output_mask", output_mask));

    float out1_reduction = 0;
    try
    {
      out1_reduction = rcg::getFloat(nodemap, "ChunkRcOut1Reduction", 0, 0, true);
    }
    catch (const std::exception&)
    {
      // can be removed if sensor version must be >= 20.10.1
      out1_reduction = rcg::getFloat(nodemap, "ChunkRcAdaptiveOut1Reduction", 0, 0, false);
    }
    param.extra_data.push_back(getKeyValue("out1_reduction", out1_reduction));

    try
    {
      bool adapting = rcg::getBoolean(nodemap, "ChunkRcAutoExposureAdapting", true);
      param.extra_data.push_back(getKeyValue("auto_exposure_adapting", adapting));
    }
    catch (const std::exception&)
    {
      // feature not available on device
    }

    param.extra_data.push_back(getKeyValueString("model_name", rcg::getString(nodemap, "DeviceModelName", false)));

    pub->publish(param);
  }
}

}  // namespace rc
