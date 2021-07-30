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

#include "camera_info_publisher.hpp"

#include <rc_genicam_api/pixel_formats.h>
#include <rc_genicam_api/config.h>

namespace rc
{

CameraInfoPublisher::CameraInfoPublisher(
  rclcpp::Node * node, const std::string & frame_id,
  bool _left)
: GenICam2RosPublisher(frame_id)
{
  // prepare camera info message

  info.header.frame_id = frame_id;

  info.width = 0;
  info.height = 0;
  info.distortion_model = "plumb_bob";  // we have to choose a model
  info.d.resize(5);                     // all 0, since images are rectified

  info.k[0] = 1;
  info.k[1] = 0;
  info.k[2] = 0;
  info.k[3] = 0;
  info.k[4] = 1;
  info.k[5] = 0;
  info.k[6] = 0;
  info.k[7] = 0;
  info.k[8] = 1;

  info.r[0] = 1;
  info.r[1] = 0;
  info.r[2] = 0;
  info.r[3] = 0;
  info.r[4] = 1;
  info.r[5] = 0;
  info.r[6] = 0;
  info.r[7] = 0;
  info.r[8] = 1;

  info.p[0] = 1;
  info.p[1] = 0;
  info.p[2] = 0;
  info.p[3] = 0;
  info.p[4] = 0;
  info.p[5] = 1;
  info.p[6] = 0;
  info.p[7] = 0;
  info.p[8] = 0;
  info.p[9] = 0;
  info.p[10] = 1;
  info.p[11] = 0;

  info.binning_x = 1;
  info.binning_y = 1;

  // advertise topic

  left = _left;

  if (left) {
    pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("stereo/left/camera_info", 1);
    left = true;
  } else {
    pub = node->create_publisher<sensor_msgs::msg::CameraInfo>("stereo/right/camera_info", 1);
    left = false;
  }
}

bool CameraInfoPublisher::used()
{
  return pub->get_subscription_count() > 0;
}

void CameraInfoPublisher::requiresComponents(int & components, bool &)
{
  if (pub->get_subscription_count() > 0) {
    components |= ComponentIntensity;
  }
}

void CameraInfoPublisher::publish(const rcg::Buffer * buffer, uint32_t part, uint64_t pixelformat)
{
  if (nodemap && pub->get_subscription_count() > 0 && (pixelformat == Mono8 ||
    pixelformat == YCbCr411_8 || pixelformat == RGB8))
  {
    uint64_t time = buffer->getTimestampNS();

    info.header.stamp.sec = time / 1000000000ul;
    info.header.stamp.nanosec = time % 1000000000ul;

    info.width = static_cast<uint32_t>(buffer->getWidth(part));
    info.height = static_cast<uint32_t>(buffer->getHeight(part));

    if (info.height > info.width) {
      info.height >>= 1;  // left and right images are stacked together
      rcg::setEnum(nodemap, "ChunkComponentSelector", "IntensityCombined", false);
    } else {
      rcg::setEnum(nodemap, "ChunkComponentSelector", "Intensity", true);
    }

    double f = rcg::getFloat(nodemap, "ChunkScan3dFocalLength", 0, 0, true);
    double t = rcg::getFloat(nodemap, "ChunkScan3dBaseline", 0, 0, true);

    info.k[0] = info.k[4] = f;
    info.p[0] = info.p[5] = f;

    info.p[2] = info.k[2] = rcg::getFloat(nodemap, "ChunkScan3dPrincipalPointU", 0, 0, true);
    info.p[6] = info.k[5] = rcg::getFloat(nodemap, "ChunkScan3dPrincipalPointV", 0, 0, true);

    if (left) {
      info.p[3] = 0;
    } else {
      info.p[3] = -f * t;
    }

    pub->publish(info);
  }
}

}  // namespace rc
