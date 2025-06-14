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

#include "image_publisher.hpp"

#include <rc_genicam_api/image.h>
#include <rc_genicam_api/pixel_formats.h>
#include <rc_genicam_api/config.h>

#include <sensor_msgs/image_encodings.hpp>

namespace rc
{

ImagePublisher::ImagePublisher(
  rclcpp::Node * node, const std::string & frame_id, bool _left,
  bool _color, bool out1_filter)
: GenICam2RosPublisher(frame_id)
{
  left = _left;
  color = _color;

  std::string name;

  if (left) {
    name = "stereo/left/image_rect";
  } else {
    name = "stereo/right/image_rect";
  }

  if (color) {
    name = name + "_color";
  }

  pub = image_transport::create_publisher(node, name);

  if (out1_filter) {
    pub_out1_low = image_transport::create_publisher(node, name + "_out1_low");
    pub_out1_high = image_transport::create_publisher(node, name + "_out1_high");
  }
}

bool ImagePublisher::used()
{
  return pub.getNumSubscribers() > 0 || pub_out1_low.getNumSubscribers() > 0 ||
         pub_out1_high.getNumSubscribers() > 0;
}

void ImagePublisher::requiresComponents(int & components, bool & _color)
{
  if (pub.getNumSubscribers() > 0 || pub_out1_low.getNumSubscribers() > 0 ||
    pub_out1_high.getNumSubscribers() > 0)
  {
    if (left) {
      components |= ComponentIntensity;
    } else {
      components |= ComponentIntensityCombined;
    }

    if (color) {
      _color = true;
    }
  }
}

void ImagePublisher::publish(const rcg::Buffer * buffer, uint32_t part, uint64_t pixelformat)
{
  if (nodemap) {
    rcg::setEnum(nodemap, "ChunkLineSelector", "Out1", true);
    bool out1 = rcg::getInteger(nodemap, "ChunkLineStatusAll", 0, 0, true) & 0x1;

    bool sub = (pub.getNumSubscribers() > 0);

    if (!out1 && pub_out1_low.getNumSubscribers() > 0) {
      sub = true;
    }
    if (out1 && pub_out1_high.getNumSubscribers() > 0) {
      sub = true;
    }

    if (sub && (pixelformat == Mono8 || pixelformat == YCbCr411_8 || pixelformat == RGB8)) {
      // create image and initialize header

      std::shared_ptr<sensor_msgs::msg::Image> im = std::make_shared<sensor_msgs::msg::Image>();

      uint64_t time = buffer->getTimestampNS();

      im->header.stamp.sec = time / 1000000000ul;
      im->header.stamp.nanosec = time % 1000000000ul;
      im->header.frame_id = frame_id;

      // set image size

      im->width = static_cast<uint32_t>(buffer->getWidth(part));
      im->height = static_cast<uint32_t>(buffer->getHeight(part));
      im->is_bigendian = false;

      bool stacked = false;

      if (im->height > im->width) {
        stacked = true;
        im->height >>= 1;
      }

      // get pointer to image data in buffer

      const uint8_t * ps = static_cast<const uint8_t *>(buffer->getBase(part));
      size_t pstep = im->width + buffer->getXPadding(part);

      if (pixelformat == YCbCr411_8) {
        pstep = (im->width >> 2) * 6 + buffer->getXPadding(part);
      }
      else if (pixelformat == RGB8)
      {
        pstep = 3*im->width + buffer->getXPadding(part);
      }


      if (!left) {
        if (stacked) {
          ps += pstep * im->height;
        } else {
          return;  // buffer does not contain a right image
        }
      }

      // convert image data

      if (color) { // convert to color
        im->encoding = sensor_msgs::image_encodings::RGB8;
        im->step = 3 * im->width * sizeof(uint8_t);

        im->data.resize(im->step * im->height);
        uint8_t * pt = reinterpret_cast<uint8_t *>(&im->data[0]);

        if (pixelformat == Mono8) // convert from monochrome
        {
          return;  // do not convert from monochrome, skip instead
        }
        else if (pixelformat == YCbCr411_8) // convert from YUV 411
        {
          for (uint32_t k = 0; k < im->height; k++) {
            for (uint32_t i = 0; i < im->width; i += 4) {
              rcg::convYCbCr411toQuadRGB(pt, ps, i);
              pt += 12;
            }
            ps += pstep;
          }
        }
        else if (pixelformat == RGB8)
        {
          for (uint32_t k = 0; k < im->height; k++)
          {
            for (uint32_t i = 0; i < im->width; i++)
            {
              *pt++ = *ps++;
              *pt++ = *ps++;
              *pt++ = *ps++;
            }

            ps += buffer->getXPadding(part);
          }
        }

      } else { // convert to monochrome
        im->encoding = sensor_msgs::image_encodings::MONO8;
        im->step = im->width * sizeof(uint8_t);

        im->data.resize(im->step * im->height);
        uint8_t * pt = reinterpret_cast<uint8_t *>(&im->data[0]);

        if (pixelformat == Mono8) { // copy monochrome image
          for (uint32_t k = 0; k < im->height; k++) {
            for (uint32_t i = 0; i < im->width; i++) {
              *pt++ = ps[i];
            }

            ps += pstep;
          }
        }
        else if (pixelformat == YCbCr411_8) // copy monochrome part of YUV 411 image
        {
          for (uint32_t k = 0; k < im->height; k++) {
            int j = 0;

            for (uint32_t i = 0; i < im->width; i += 4) {
              *pt++ = ps[j];
              *pt++ = ps[j + 1];
              *pt++ = ps[j + 3];
              *pt++ = ps[j + 4];
              j += 6;
            }

            ps += pstep;
          }
        }
        else if (pixelformat == RGB8)
        {
          for (uint32_t k = 0; k < im->height; k++)
          {
            for (uint32_t i = 0; i < im->width; i++)
            {
              *pt++ = static_cast<uint8_t>((9798*ps[0]+19234*ps[1]+3736*ps[2]+16384)>>15);
              ps += 3;
            }
            ps += buffer->getXPadding(part);
          }
        }

      }

      // publish message

      pub.publish(im);

      if (pub_out1_low && !out1) {
        pub_out1_low.publish(im);
      }

      if (pub_out1_high && out1) {
        pub_out1_high.publish(im);
      }
    }
  }
}

}  // namespace rc
