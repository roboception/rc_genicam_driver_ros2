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

#include "points2_publisher.hpp"

#include <rc_genicam_api/pixel_formats.h>
#include <rc_genicam_api/config.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rc
{

Points2Publisher::Points2Publisher(rclcpp::Node * _node, const std::string & frame_id)
: GenICam2RosPublisher(frame_id), left_list(75)
{
  f = 0;
  t = 0;
  invalid = -1;
  scale = 1;

  node = _node;
  pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("stereo/points2", 1);
}

bool Points2Publisher::used()
{
  return pub->get_subscription_count() > 0;
}

void Points2Publisher::requiresComponents(int & components, bool &)
{
  if (pub->get_subscription_count() > 0) {
    components |= ComponentIntensity | ComponentDisparity;
  }
}

void Points2Publisher::publish(const rcg::Buffer * buffer, uint32_t part, uint64_t pixelformat)
{
  if (nodemap) {
    if (pub->get_subscription_count() > 0) {
      rcg::setEnum(nodemap, "ChunkLineSelector", "Out1", true);
      std::string out1_mode = rcg::getEnum(nodemap, "ChunkLineSource", true);
      uint64_t tolerance_ns;

      if (out1_mode == "ExposureAlternateActive") {
        tolerance_ns = static_cast<uint64_t>(0.050 * 1000000000ull);
      } else {
        tolerance_ns = 0;
      }

      bool out1 = rcg::getInteger(nodemap, "ChunkLineStatusAll", 0, 0, true) & 0x1;

      // buffer left and disparity images

      if (pixelformat == Mono8 || pixelformat == YCbCr411_8 || pixelformat == RGB8) {
        // in alternate exposure mode, skip images for texture with out1 == true,
        // i.e. with projected pattern

        if (tolerance_ns > 0 && out1) {
          return;
        }

        left_list.add(buffer, part);
      } else if (pixelformat == Coord3D_C16) {
        disp_list.add(buffer, part);

        rcg::setEnum(nodemap, "ChunkComponentSelector", "Disparity", true);
        f = rcg::getFloat(nodemap, "ChunkScan3dFocalLength", 0, 0, true);
        t = rcg::getFloat(nodemap, "ChunkScan3dBaseline", 0, 0, true);

        invalid = -1;
        if (rcg::getBoolean(nodemap, "ChunkScan3dInvalidDataFlag", false)) {
          invalid = rcg::getFloat(nodemap, "ChunkScan3dInvalidDataValue", 0, 0, true);
        }

        scale = rcg::getFloat(nodemap, "ChunkScan3dCoordinateScale", 0, 0, true);
      }

      // get corresponding left and disparity image

      uint64_t timestamp = buffer->getTimestampNS();

      std::shared_ptr<const rcg::Image> left = left_list.find(timestamp, tolerance_ns);
      std::shared_ptr<const rcg::Image> disp = disp_list.find(timestamp, tolerance_ns);

      // print warning with reason if no left image can be found for disparity image

      if (pixelformat == Coord3D_C16 && !left) {
        RCLCPP_WARN(node->get_logger(), "Cannot find left image for disparity image.");
      }

      if (left && disp) {
        // determine integer factor between size of left and disparity image

        uint32_t lw = left->getWidth();
        uint32_t lh = left->getHeight();

        if (lh > lw) { // there may be a stacked right image
          lh >>= 1;
        }

        int ds = (lw + disp->getWidth() - 1) / disp->getWidth();

        if ((lw + ds - 1) / ds == disp->getWidth() && (lh + ds - 1) / ds == disp->getHeight()) {
          // allocate new image message and set meta information

          std::unique_ptr<sensor_msgs::msg::PointCloud2> p =
            std::make_unique<sensor_msgs::msg::PointCloud2>();

          p->header.stamp.sec = timestamp / 1000000000ul;
          p->header.stamp.nanosec = timestamp % 1000000000ul;
          p->header.frame_id = frame_id;

          // set meta data of point cloud

          p->width = lw / ds;   // consider only full pixels if downscaled
          p->height = lh / ds;  // consider only full pixels if downscaled

          p->is_bigendian = rcg::isHostBigEndian();
          p->is_dense = false;

          p->fields.resize(4);
          p->fields[0].name = "x";
          p->fields[0].offset = 0;
          p->fields[0].count = 1;
          p->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
          p->fields[1].name = "y";
          p->fields[1].offset = 4;
          p->fields[1].count = 1;
          p->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
          p->fields[2].name = "z";
          p->fields[2].offset = 8;
          p->fields[2].count = 1;
          p->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
          p->fields[3].name = "rgb";
          p->fields[3].offset = 12;
          p->fields[3].count = 1;
          p->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;

          p->point_step = 16;
          p->row_step = p->point_step * p->width;

          // allocate memory

          p->data.resize(p->row_step * p->height);
          float * pd = reinterpret_cast<float *>(&p->data[0]);

          // pointer to disparity data

          const uint8_t * dps = disp->getPixels();
          size_t dstep = disp->getWidth() * sizeof(uint16_t) + disp->getXPadding();

          // convert disparity to point cloud using left image for texture

          bool bigendian = disp->isBigEndian();

          for (uint32_t k = 0; k < p->height; k++) {
            for (uint32_t i = 0; i < p->width; i++) {
              // get disparity

              uint32_t j = i << 1;

              float d;

              if (bigendian) {
                d = scale * ((dps[j] << 8) | dps[j + 1]);
              } else {
                d = scale * ((dps[j + 1] << 8) | dps[j]);
              }

              // if disparity is valid and color can be obtained

              if (d > 0 && d != invalid) {
                // reconstruct 3D point

                pd[0] = (i + 0.5 - disp->getWidth() / 2.0) * t / d;
                pd[1] = (k + 0.5 - disp->getHeight() / 2.0) * t / d;
                pd[2] = f * t / d;

                // store color of point

                uint8_t rgb[3];
                rcg::getColor(rgb, left, ds, i, k);

                uint8_t * bgra = reinterpret_cast<uint8_t *>(pd + 3);

                bgra[0] = rgb[2];
                bgra[1] = rgb[1];
                bgra[2] = rgb[0];
                bgra[3] = 0;
              } else {
                for (int i = 0; i < 4; i++) {
                  pd[i] = std::numeric_limits<float>::quiet_NaN();
                }
              }

              pd += 4;
            }

            dps += dstep;
          }

          // publish message

          pub->publish(std::move(p));
        } else {
          RCLCPP_ERROR_STREAM(
            node->get_logger(), "Size of left and disparity image must differ only by an integer factor: " <<
              left->getWidth() << "x" << left->getHeight() << " != " << disp->getWidth() << "x" <<
              disp->getHeight());
        }

        // remove all old images, including the current ones

        left_list.removeOld(timestamp);
        disp_list.removeOld(timestamp);
      }
    }
  }
}

}  // namespace rc
