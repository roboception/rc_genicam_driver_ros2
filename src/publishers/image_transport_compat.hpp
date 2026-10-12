/*
 * Copyright (c) 2026 Roboception GmbH
 * All rights reserved
 *
 * Author: Felix Ruess
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

#ifndef RC_IMAGE_TRANSPORT_COMPAT_H
#define RC_IMAGE_TRANSPORT_COMPAT_H

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

namespace rc
{

/**
 * Creates an image_transport publisher with default QoS (keep last 10,
 * reliable, volatile).
 *
 * image_transport >= 6.4 takes node interfaces and an rclcpp::QoS, and
 * 7.0.1 removed the overload taking rclcpp::Node * and rmw_qos_profile_t.
 * Older distributions (humble, jazzy, kilted) only have the latter.
 */

inline image_transport::Publisher createImagePublisher(
  rclcpp::Node * node,
  const std::string & topic)
{
#ifdef IMAGE_TRANSPORT_HAS_NODE_INTERFACES
  return image_transport::create_publisher(*node, topic, rclcpp::QoS(10));
#else
  return image_transport::create_publisher(node, topic);
#endif
}

}  // namespace rc

#endif
