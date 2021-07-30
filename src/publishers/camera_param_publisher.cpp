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

    // get list of all available IO lines and iterate over them to get their
    // LineSource values

    std::vector<std::string> lines;
    rcg::getEnum(nodemap, "ChunkLineSelector", lines, true);
    for (auto && line : lines) {
      rcg::setEnum(nodemap, "ChunkLineSelector", line.c_str(), true);

      rc_common_msgs::msg::KeyValue line_source;
      line_source.key = line;
      line_source.value = rcg::getEnum(nodemap, "ChunkLineSource", true);

      param.line_source.push_back(line_source);
    }

    param.line_status_all = rcg::getInteger(nodemap, "ChunkLineStatusAll", 0, 0, true);

    param.gain = rcg::getFloat(nodemap, "ChunkGain", 0, 0, true);
    param.exposure_time = rcg::getFloat(nodemap, "ChunkExposureTime", 0, 0, true) / 1000000l;

    {
      float noise = rcg::getFloat(nodemap, "ChunkRcNoise", 0, 0, false);
      bool test = rcg::getBoolean(nodemap, "ChunkRcTest", false);

      float out1_reduction = 0;

      try {
        out1_reduction = rcg::getFloat(nodemap, "ChunkRcOut1Reduction", 0, 0, true);
      } catch (const std::exception &) {
        // can be removed if sensor version must be >= 20.10.1
        out1_reduction = rcg::getFloat(nodemap, "ChunkRcAdaptiveOut1Reduction", 0, 0, false);
      }

      float brightness = rcg::getFloat(nodemap, "ChunkRcBrightness", 0, 0, false);

      param.extra_data.clear();
      param.extra_data.push_back(getKeyValue("noise", noise));
      param.extra_data.push_back(getKeyValue("test", test));
      param.extra_data.push_back(getKeyValue("out1_reduction", out1_reduction));
      param.extra_data.push_back(getKeyValue("brightness", brightness));
    }

    pub->publish(param);
  }
}

}  // namespace rc
