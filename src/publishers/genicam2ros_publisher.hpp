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

#ifndef RC_GENICAM2ROS_PUBLISHER_H
#define RC_GENICAM2ROS_PUBLISHER_H

#include <string>

#include <GenApi/GenApi.h>
#include <rc_genicam_api/buffer.h>

namespace rc
{
/**
 * Interface for all publishers relating to images, point clouds or
 * other stereo-camera data
 */

class GenICam2RosPublisher
{
public:
  /*
    Flags for components to be enabled.
  */

  const static int ComponentIntensity = 1;
  const static int ComponentIntensityCombined = 2;
  const static int ComponentDisparity = 4;
  const static int ComponentConfidence = 8;
  const static int ComponentError = 16;

  /**
   * @param frame_id_prefix prefix for frame ids in published ros messages
   */

  GenICam2RosPublisher(const std::string & _frame_id)
  : frame_id(_frame_id)
  {}

  virtual ~GenICam2RosPublisher()
  {}

  /**
    Set nodemap to be used.
  */

  void setNodemap(const std::shared_ptr<GenApi::CNodeMapRef> & _nodemap)
  {
    nodemap = _nodemap;
  }

  /**
    Clear nodemap.
  */

  void clearNodemap()
  {
    nodemap.reset();
  }

  /**
    Offers a buffer for publication. It depends on the the kind of buffer
    data and the implementation and configuration of the sub-class if the
    data is published.

    @param buffer      Buffer with data to be published. The buffer is
                       already attached to the nodemap for accessing the
                       chunk data.
    @param part        Part index of image.
    @param pixelformat The pixelformat as given by buffer->getPixelFormat(part).
  */

  virtual void publish(const rcg::Buffer * buffer, uint32_t part, uint64_t pixelformat) = 0;

  /**
    Returns true if there are subscribers to the topic.

    @return True if there are subscribers.
  */

  virtual bool used() = 0;

  /**
    Adds components and if color images are required to the given values.
    Nothing will be changed if there are no subscribers, i.e. used() == false.

    @param components Components Flags that will be updated according to the
                      needs of this publisher.
    @param color      Value that will be updated if this publisher needs color.
  */

  virtual void requiresComponents(int & components, bool & color) = 0;

protected:
  std::string frame_id;
  std::shared_ptr<GenApi::CNodeMapRef> nodemap;

private:
  GenICam2RosPublisher & operator=(const GenICam2RosPublisher &);  // forbidden
};

}  // namespace rc

#endif
