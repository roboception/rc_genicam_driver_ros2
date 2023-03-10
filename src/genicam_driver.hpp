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

#ifndef RC_GENICAM_DRIVER_H
#define RC_GENICAM_DRIVER_H

#include "publishers/genicam2ros_publisher.hpp"
#include "visibility.h"

#include <rclcpp/rclcpp.hpp>

#include <rc_common_msgs/srv/trigger.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <GenApi/GenApi.h>
#include <rc_genicam_api/device.h>

#include <vector>
#include <map>
#include <string>
#include <utility>
#include <thread>
#include <mutex>
#include <atomic>

namespace rc
{

class GenICamDriver : public rclcpp::Node
{
public:
  RC_COMPOSITION_PUBLIC
  GenICamDriver(const rclcpp::NodeOptions & options);
  virtual ~GenICamDriver();

//    bool depthAcquisitionTrigger(rc_common_msgs::Trigger::Request& req, rc_common_msgs::Trigger::Response& resp);

private:
  bool declareGenICamParameter(
    const std::string & ros_name,
    const std::shared_ptr<GenApi::CNodeMapRef> & nodemap, const std::string & name,
    const char *description=0, double float_scale=1.0);

  bool declareGenICamParameter(
    const std::string & ros_name,
    const std::shared_ptr<GenApi::CNodeMapRef> & nodemap, const std::string & name,
    const std::string & selector_name, const std::string & selector_value,
    const char *description=0, double float_scale=1.0);

  void configure();
  void cleanup();
  void updateSubscriptions(bool force);
  void checkSubscriptions();

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & params);

  void triggerDepthAcquisition(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<rc_common_msgs::srv::Trigger::Request>,
    std::shared_ptr<rc_common_msgs::srv::Trigger::Response> res);

  void publishConnectionDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void publishDeviceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void process();

  diagnostic_updater::Updater updater;

  std::string frame_id;

  std::shared_ptr<rcg::Device> dev;
  std::shared_ptr<GenApi::CNodeMapRef> nodemap;

  std::recursive_mutex param_mtx;
  std::map<std::string, std::string> param;
  std::map<std::string, double> param_float_scale;
  std::map<std::string, std::pair<std::string, std::string>> param_selector;
  OnSetParametersCallbackHandle::SharedPtr param_cb;

  std::string color_format;
  int scomponents;
  bool scolor;

  std::thread process_thread;
  std::atomic_bool running;

  std::vector<std::shared_ptr<GenICam2RosPublisher>> pub;
  rclcpp::TimerBase::SharedPtr pub_sub_timer;

  rclcpp::Service<rc_common_msgs::srv::Trigger>::SharedPtr trigger_service;

  bool iocontrol_avail;
  std::string remote_out1_mode;
  bool update_exp_values;
  bool update_wb_values;

  std::recursive_mutex updater_mtx;
  std::string device_model;
  std::string device_version;
  std::string device_serial;
  std::string device_mac;
  std::string device_name;
  std::string device_interface;
  std::string device_ip;
  int gev_packet_size;
  int connection_loss_total;
  int complete_buffers_total;
  int incomplete_buffers_total;
  int image_receive_timeouts_total;
  int current_reconnect_trial;
  bool streaming;
};

} // namespace rc

#endif
