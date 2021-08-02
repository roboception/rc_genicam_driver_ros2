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

#include "genicam_driver.hpp"
#include "publishers/camera_info_publisher.hpp"
#include "publishers/camera_param_publisher.hpp"
#include "publishers/image_publisher.hpp"
#include "publishers/disparity_publisher.hpp"
#include "publishers/disparity_color_publisher.hpp"
#include "publishers/depth_publisher.hpp"
#include "publishers/confidence_publisher.hpp"
#include "publishers/error_disparity_publisher.hpp"
#include "publishers/error_depth_publisher.hpp"
#include "publishers/points2_publisher.hpp"

#include <rc_genicam_api/device.h>
#include <rc_genicam_api/stream.h>
#include <rc_genicam_api/buffer.h>
#include <rc_genicam_api/config.h>
#include <rc_genicam_api/pixel_formats.h>

#include <exception>

#include <sstream>
#include <stdexcept>
#include <chrono>
#include <unistd.h>

#include <rc_common_msgs/msg/return_code_constants.hpp>

namespace rc
{

GenICamDriver::GenICamDriver(const rclcpp::NodeOptions & options)
: Node("genicam_driver", options), updater(this)
{
  RCLCPP_INFO(this->get_logger(), "Initializing");

  // initialize member variables

  scomponents = 0;
  scolor = 0;

  running = false;

  gev_packet_size = 0;
  connection_loss_total = 0;
  complete_buffers_total = 0;
  incomplete_buffers_total = 0;
  image_receive_timeouts_total = 0;
  current_reconnect_trial = 0;
  streaming = false;

  // define frame id

  std::string ns = get_namespace();

  if (ns.size() > 0 && ns[0] == '/') {
    ns = ns.substr(1);
  }

  if (ns.size() > 0) {
    frame_id = ns + "_camera";
  } else {
    frame_id = "camera";
  }

  // declare read-only parameters

  rcl_interfaces::msg::ParameterDescriptor device_descr;
  device_descr.description = "Device ID which can be '*' (default) if only one device is connected";
  device_descr.additional_constraints = "[[<interface>]:]<serial>|<name>";
  device_descr.read_only = true;

  declare_parameter("device", std::string("*"), device_descr);

  // add callbacks for diagnostics publishing

  updater.add("Connection", this, &GenICamDriver::publishConnectionDiagnostics);
  updater.add("Device", this, &GenICamDriver::publishDeviceDiagnostics);

  // start grabbing thread

  running = true;
  process_thread = std::thread(&GenICamDriver::process, this);
}

GenICamDriver::~GenICamDriver()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down");

  // signal running thread to stop and wait until it has finished

  running = false;
  if (process_thread.joinable()) {
    process_thread.join();
  }

  rcg::System::clearSystems();
}

namespace
{

std::vector<std::shared_ptr<rcg::Device>> getSupportedDevices(
  const std::string & devid,
  const std::vector<std::string> & iname)
{
  std::vector<std::shared_ptr<rcg::System>> system = rcg::System::getSystems();
  std::vector<std::shared_ptr<rcg::Device>> ret;

  for (size_t i = 0; i < system.size(); i++) {
    system[i]->open();

    std::vector<std::shared_ptr<rcg::Interface>> interf = system[i]->getInterfaces();

    for (size_t k = 0; k < interf.size(); k++) {
      if (interf[k]->getTLType() == "GEV" &&
        (iname.size() == 0 ||
        std::find(iname.begin(), iname.end(), interf[k]->getID()) != iname.end()))
      {
        interf[k]->open();

        std::vector<std::shared_ptr<rcg::Device>> device = interf[k]->getDevices();

        for (size_t j = 0; j < device.size(); j++) {
          if ((device[j]->getVendor() == "Roboception GmbH" ||
            device[j]->getModel().substr(0,
            9) == "rc_visard" || device[j]->getModel().substr(0, 7) == "rc_cube") &&
            (devid == "*" || device[j]->getID() == devid || device[j]->getSerialNumber() == devid ||
            device[j]->getDisplayName() == devid))
          {
            ret.push_back(device[j]);
          }
        }

        interf[k]->close();
      }
    }

    system[i]->close();
  }

  return ret;
}

class NoDeviceException : public std::invalid_argument
{
public:
  NoDeviceException(const char * msg)
  : std::invalid_argument(msg)
  {}
};

void split(
  std::vector<std::string> & list, const std::string & s, char delim,
  bool skip_empty = true)
{
  std::stringstream in(s);
  std::string elem;

  while (getline(in, elem, delim)) {
    if (!skip_empty || elem.size() > 0) {
      list.push_back(elem);
    }
  }
}

}  // namespace

bool GenICamDriver::declareGenICamParameter(
  const std::string & ros_name,
  const std::shared_ptr<GenApi::CNodeMapRef> & nodemap, const std::string & name)
{
  bool ret = false;

  try {
    std::lock_guard<std::recursive_mutex> lock(param_mtx);

    GenApi::INode * node = nodemap->_GetNode(name.c_str());

    if (node != 0) {
      if (GenApi::IsReadable(node) && GenApi::IsWritable(node)) {
        rcl_interfaces::msg::ParameterDescriptor param_descr;
        param_descr.description = node->GetDescription();

        switch (node->GetPrincipalInterfaceType()) {
          case GenApi::intfIBoolean:
            {
              GenApi::IBoolean * p = dynamic_cast<GenApi::IBoolean *>(node);

              param[ros_name] = name;
              declare_parameter(ros_name, p->GetValue(false, false), param_descr);
              ret = true;
            }
            break;

          case GenApi::intfIInteger:
            {
              GenApi::IInteger * p = dynamic_cast<GenApi::IInteger *>(node);

              rcl_interfaces::msg::IntegerRange int_range;

              int_range.from_value = p->GetMin();
              int_range.to_value = p->GetMax();

              int_range.step = 1;
              if (p->GetIncMode() == GenApi::fixedIncrement) {
                int_range.step = p->GetInc();
              }

              param_descr.integer_range.push_back(int_range);

              param[ros_name] = name;
              declare_parameter(ros_name, p->GetValue(false, false), param_descr);
              ret = true;
            }
            break;

          case GenApi::intfIFloat:
            {
              GenApi::IFloat * p = dynamic_cast<GenApi::IFloat *>(node);

              rcl_interfaces::msg::FloatingPointRange float_range;

              float_range.from_value = p->GetMin();
              float_range.to_value = p->GetMax();

              float_range.step = 0;
              if (p->GetIncMode() == GenApi::fixedIncrement) {
                float_range.step = p->GetInc();
              }

              param_descr.floating_point_range.push_back(float_range);

              param[ros_name] = name;
              declare_parameter(ros_name, p->GetValue(false, false), param_descr);
              ret = true;
            }
            break;

          case GenApi::intfIString:
            {
              GenApi::IString * p = dynamic_cast<GenApi::IString *>(node);

              param[ros_name] = name;
              declare_parameter(ros_name, std::string(p->GetValue(false, false)), param_descr);
              ret = true;
            }
            break;

          case GenApi::intfIEnumeration:
            {
              GenApi::IEnumeration * p = dynamic_cast<GenApi::IEnumeration *>(node);

              GenApi::StringList_t entries;
              p->GetSymbolics(entries);

              std::ostringstream out;
              for (size_t i = 0; i < entries.size(); i++) {
                if (i > 0) {out << '|';}
                out << entries[i];
              }

              param_descr.additional_constraints = out.str();

              std::string val;
              GenApi::IEnumEntry * entry = p->GetCurrentEntry();
              if (entry != 0) {
                val = entry->GetSymbolic();
              }

              param[ros_name] = name;
              declare_parameter(ros_name, val, param_descr);
              ret = true;
            }
            break;

          default:
            RCLCPP_INFO_STREAM(
              this->get_logger(),
              "Parameter has unsupported type: " << ros_name << " (" << name << ")");
            break;
        }
      } else {
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "Parameter not readable or writable: " << ros_name << " (" << name << ")");
      }
    } else {
      RCLCPP_INFO_STREAM(
        this->get_logger(), "Parameter does not exists: " << ros_name << " (" << name << ")");
    }
  } catch (const GENICAM_NAMESPACE::GenericException & ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter: " << ros_name << " (" << name << ")");
  }

  return ret;
}

bool GenICamDriver::declareGenICamParameter(
  const std::string & ros_name,
  const std::shared_ptr<GenApi::CNodeMapRef> & nodemap, const std::string & name,
  const std::string & selector_name, const std::string & selector_value)
{
  std::lock_guard<std::recursive_mutex> lock(param_mtx);
  bool ret = false;

  // set selector as requested

  std::string v = rcg::getEnum(nodemap, selector_name.c_str(), false);

  if (v == selector_value || rcg::setEnum(nodemap, selector_name.c_str(), selector_value.c_str(),
    false))
  {
    param_selector[ros_name] = std::make_pair(selector_name, selector_value);

    // declare paraemter

    ret = declareGenICamParameter(ros_name, nodemap, name);
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Selector of parameter cannot be found or changed: " <<
      ros_name << " (" << selector_name << "=" << selector_value << ")");
  }

  return ret;
}

void GenICamDriver::configure()
{
  using namespace std::placeholders;

  RCLCPP_INFO(this->get_logger(), "Configuring");

  {
    std::lock_guard<std::recursive_mutex> lock(updater_mtx);

    device_model = "";
    device_version = "";
    device_serial = "";
    device_mac = "";
    device_name = "";
    device_interface = "";
    device_ip = "";
    gev_packet_size = 0;
    current_reconnect_trial = 1;
  }

  // get device id from read only parameter

  std::string id;
  get_parameter("device", id);

  std::vector<std::string> iname;  // empty
  std::string dname = id;

  {
    size_t i = dname.find(':');
    if (i != std::string::npos) {
      if (i > 0) {
        iname.push_back(id.substr(0, i));
      }

      dname = dname.substr(i + 1);
    }
  }

  // open device and get nodemap

  std::vector<std::shared_ptr<rcg::Device>> devices = getSupportedDevices(dname, iname);

  if (devices.size() == 0) {
    throw NoDeviceException(("Cannot find device '" + id + "'").c_str());
  }

  if (devices.size() > 1) {
    throw std::invalid_argument("Too many devices, please specify unique ID");
  }

  dev = devices[0];
  dev->open(rcg::Device::CONTROL);
  nodemap = dev->getRemoteNodeMap();

  // check if device is ready

  if (!rcg::getBoolean(nodemap, "RcSystemReady", true, true)) {
    throw std::invalid_argument("Device is not yet ready");
  }

  {
    std::lock_guard<std::recursive_mutex> lock(updater_mtx);

    // get serial number and IP

    device_interface = dev->getParent()->getID();
    device_serial = dev->getSerialNumber();
    device_mac = rcg::getString(nodemap, "GevMACAddress", true);
    device_name = rcg::getString(nodemap, "DeviceUserID", true);
    device_ip = rcg::getString(nodemap, "GevCurrentIPAddress", true);

    updater.setHardwareID(device_serial);

    RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to sensor '" << device_interface << ":" <<
      device_serial << "' alias " << dev->getDisplayName());

    // ensure that device version >= 20.04

    device_version = rcg::getString(nodemap, "DeviceVersion");

    std::vector<std::string> list;
    split(list, device_version, '.');

    if (list.size() < 3 || std::stoi(list[0]) < 20 ||
      (std::stoi(list[0]) == 20 && std::stoi(list[1]) < 4))
    {
      running = false;
      throw std::invalid_argument("Device version must be 20.04 or higher: " + device_version);
    }

    // get model type of the device

    device_model = rcg::getString(nodemap, "DeviceModelName");
  }

  // check for color sensor and iocontrol

  bool color = false;

  {
    std::vector<std::string> formats;
    rcg::setEnum(nodemap, "ComponentSelector", "Intensity", true);
    rcg::getEnum(nodemap, "PixelFormat", formats, true);
    for (auto && format : formats) {
      if (format == "YCbCr411_8")
      {
        color_format = "YCbCr411_8";
        color = true;
        break;
      }
      if (format == "RGB8")
      {
        color_format = "RGB8";
        color = true;
        break;
      }
    }
  }

  bool iocontrol_avail = nodemap->_GetNode("LineSource")->GetAccessMode() == GenApi::RW;

  // initialise variables for caching some values

  remote_out1_mode = "";
  update_exp_values = false;
  update_wb_values = false;

  // register parameter callback

  param_cb = add_on_set_parameters_callback(std::bind(&GenICamDriver::paramCallback, this, _1));

  // declare parameters and mapping to GenICam (all are allowed to fail if
  // parameters are not available in the given nodemap)

  // TODO: This could be made more generic by loading the mapping between ROS
  // and GenICam parameters from a configuration file instead of hard coding
  // it.

  declareGenICamParameter("camera_fps", nodemap, "AcquisitionFrameRate");
  declareGenICamParameter("camera_exp_auto", nodemap, "ExposureAuto");
  declareGenICamParameter("camera_exp_max", nodemap, "ExposureTimeAutoMax");
  declareGenICamParameter("camera_exp_auto_average_max", nodemap, "RcExposureAutoAverageMax");
  declareGenICamParameter("camera_exp_auto_average_min", nodemap, "RcExposureAutoAverageMin");
  declareGenICamParameter("camera_exp_width", nodemap, "ExposureRegionWidth");
  declareGenICamParameter("camera_exp_height", nodemap, "ExposureRegionHeight");
  declareGenICamParameter("camera_exp_offset_x", nodemap, "ExposureRegionOffsetX");
  declareGenICamParameter("camera_exp_offset_y", nodemap, "ExposureRegionOffsetY");
  declareGenICamParameter("camera_exp_value", nodemap, "ExposureTime");
  declareGenICamParameter("camera_gain_value", nodemap, "Gain", "GainSelector", "All");

  if (color) {
    declareGenICamParameter("camera_wb_auto", nodemap, "BalanceWhiteAuto");
    declareGenICamParameter("camera_wb_ratio_red", nodemap, "BalanceRatio", "BalanceRatioSelector",
      "Red");
    declareGenICamParameter("camera_wb_ratio_blue", nodemap, "BalanceRatio", "BalanceRatioSelector",
      "Blue");
  }

  declareGenICamParameter("depth_acquisition_mode", nodemap, "DepthAcquisitionMode");
  declareGenICamParameter("depth_quality", nodemap, "DepthQuality");
  declareGenICamParameter("depth_static_scene", nodemap, "DepthStaticScene");
  declareGenICamParameter("depth_double_shot", nodemap, "DepthDoubleShot");
  declareGenICamParameter("depth_seg", nodemap, "DepthSeg");
  declareGenICamParameter("depth_smooth", nodemap, "DepthSmooth");
  declareGenICamParameter("depth_fill", nodemap, "DepthFill");
  declareGenICamParameter("depth_minconf", nodemap, "DepthMinConf");
  declareGenICamParameter("depth_mindepth", nodemap, "DepthMinDepth");
  declareGenICamParameter("depth_maxdepth", nodemap, "DepthMaxDepth");
  declareGenICamParameter("depth_maxdeptherr", nodemap, "DepthMaxDepthErr");

  declareGenICamParameter("ptp_enabled", nodemap, "PtpEnable");

  if (iocontrol_avail) {
    declareGenICamParameter("out1_mode", nodemap, "LineSource", "LineSelector", "Out1");
    declareGenICamParameter("out2_mode", nodemap, "LineSource", "LineSelector", "Out2");
  }

  // enable chunk data and multipart

  rcg::setEnum(nodemap, "AcquisitionAlternateFilter", "Off", false);
  rcg::setEnum(nodemap, "AcquisitionMultiPartMode", "SingleComponent", true);

  // advertise publishers

  pub.clear();
  scomponents = 0;
  scolor = false;

  pub.push_back(std::make_shared<CameraInfoPublisher>(this, frame_id, true));
  pub.push_back(std::make_shared<CameraInfoPublisher>(this, frame_id, false));
  pub.push_back(std::make_shared<CameraParamPublisher>(this, frame_id, true));
  pub.push_back(std::make_shared<CameraParamPublisher>(this, frame_id, false));

  pub.push_back(std::make_shared<ImagePublisher>(this, frame_id, true, false, iocontrol_avail));
  pub.push_back(std::make_shared<ImagePublisher>(this, frame_id, false, false, iocontrol_avail));

  if (color) {
    pub.push_back(std::make_shared<ImagePublisher>(this, frame_id, true, true, iocontrol_avail));
    pub.push_back(std::make_shared<ImagePublisher>(this, frame_id, false, true, iocontrol_avail));
  }

  pub.push_back(std::make_shared<DisparityPublisher>(this, frame_id));
  pub.push_back(std::make_shared<DisparityColorPublisher>(this, frame_id));
  pub.push_back(std::make_shared<DepthPublisher>(this, frame_id));

  pub.push_back(std::make_shared<ConfidencePublisher>(this, frame_id));
  pub.push_back(std::make_shared<ErrorDisparityPublisher>(this, frame_id));
  pub.push_back(std::make_shared<ErrorDepthPublisher>(this, frame_id));

  pub.push_back(std::make_shared<Points2Publisher>(this, frame_id));

  // make nodemap available to publshers

  for (auto && p : pub) {
    p->setNodemap(nodemap);
  }

  // update subscriptions

  updateSubscriptions(true);

  using namespace std::chrono_literals;
  pub_sub_timer = create_wall_timer(100ms, std::bind(&GenICamDriver::checkSubscriptions, this));

  // register trigger service call

  trigger_service = create_service<rc_common_msgs::srv::Trigger>(
    std::string(get_name()) + "/depth_acquisition_trigger",
    std::bind(&GenICamDriver::triggerDepthAcquisition, this, _1, _2, _3));
}

void GenICamDriver::cleanup()
{
  RCLCPP_INFO(this->get_logger(), "Cleanup");

  // remove trigger service call

  trigger_service.reset();

  // stop thread that checks for subscriptions

  if (pub_sub_timer) {
    pub_sub_timer->cancel();
    pub_sub_timer.reset();
  }

  // remove all publisher

  pub.clear();
  scomponents = 0;
  scolor = false;

  // remove parameter callback

  param_cb.reset();

  // undeclare all parameters

  for (std::map<std::string, std::string>::iterator it = param.begin(); it != param.end(); ++it) {
    try {
      undeclare_parameter(it->first);
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Cannot remove parameter: " << ex.what());
    }
  }

  param.clear();
  param_selector.clear();

  // close device and reset nodemap

  if (dev) {
    dev->close();
  }

  dev.reset();
  nodemap.reset();

  // stop publishing

  {
    std::lock_guard<std::recursive_mutex> lock(updater_mtx);

    device_model = "";
    device_version = "";
    device_serial = "";
    device_mac = "";
    device_name = "";
    device_interface = "";
    device_ip = "";
    gev_packet_size = 0;
    streaming = false;
  }
}

void GenICamDriver::updateSubscriptions(bool force)
{
  std::lock_guard<std::recursive_mutex> lock(param_mtx);

  // collect required components and color

  int rcomponents = 0;
  bool rcolor = false;

  for (auto && p : pub) {
    p->requiresComponents(rcomponents, rcolor);
  }

  // Intensity is contained in IntensityCombined

  if (rcomponents & GenICam2RosPublisher::ComponentIntensityCombined) {
    rcomponents &= ~GenICam2RosPublisher::ComponentIntensity;
  }

  // enable or disable components as required

  const static struct
  {
    const char * name;
    int flag;
  } comp[] = {{"Intensity", GenICam2RosPublisher::ComponentIntensity},
    {"IntensityCombined", GenICam2RosPublisher::ComponentIntensityCombined},
    {"Disparity", GenICam2RosPublisher::ComponentDisparity},
    {"Confidence", GenICam2RosPublisher::ComponentConfidence},
    {"Error", GenICam2RosPublisher::ComponentError},
    {0, 0}};

  for (size_t i = 0; comp[i].name != 0; i++) {
    if (((rcomponents ^ scomponents) & comp[i].flag) || force) {
      rcg::setEnum(nodemap, "ComponentSelector", comp[i].name, true);
      rcg::setBoolean(nodemap, "ComponentEnable", (rcomponents & comp[i].flag), true);

      const char * status = "disabled";
      if (rcomponents & comp[i].flag) {
        status = "enabled";
      }

      if (!force || (rcomponents & comp[i].flag)) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Component '" << comp[i].name << "' " << status);
      }
    }
  }

  // enable or disable color

    if (rcolor != scolor || force) {
    std::string format = "Mono8";
    if (rcolor) {
      format = color_format;
    }

    rcg::setEnum(nodemap, "ComponentSelector", "Intensity", true);
    rcg::setEnum(nodemap, "PixelFormat", format.c_str(), false);
    rcg::setEnum(nodemap, "ComponentSelector", "IntensityCombined", true);
    rcg::setEnum(nodemap, "PixelFormat", format.c_str(), false);
  }

  // store current settings

  scomponents = rcomponents;
  scolor = rcolor;
}

void GenICamDriver::checkSubscriptions()
{
  updateSubscriptions(false);
}

rcl_interfaces::msg::SetParametersResult GenICamDriver::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::lock_guard<std::recursive_mutex> lock(param_mtx);
  rcl_interfaces::msg::SetParametersResult ret;
  ret.successful = true;

  for (size_t i = 0; i < p.size(); i++) {
    // signal processing thread to update manual exposure or white balancing
    // values if automatic has been turned off

    if (p[i].get_name() == "camera_exp_auto" && p[i].get_type() == rclcpp::PARAMETER_STRING &&
      p[i].as_string() == "Off")
    {
      update_exp_values = true;
    }

    if (p[i].get_name() == "camera_wb_auto" && p[i].get_type() == rclcpp::PARAMETER_STRING &&
      p[i].as_string() == "Off")
    {
      update_wb_values = true;
    }

    // skip if value is cached one

    if (p[i].get_name() == "out1_mode" && p[i].get_type() == rclcpp::PARAMETER_STRING &&
      p[i].as_string() == remote_out1_mode)
    {
      continue;
    }

    // set selector if any

    try {
      const std::pair<std::string, std::string> & sel = param_selector.at(p[i].get_name());

      std::string v = rcg::getEnum(nodemap, sel.first.c_str(), false);

      if (v != sel.second && !rcg::setEnum(nodemap, sel.first.c_str(), sel.second.c_str(), false)) {
        ret.successful = false;
        ret.reason = "Cannot set selector " + sel.first + " to value " + sel.second;
        break;
      }
    } catch (const std::out_of_range &) {
      // permitted as not all parameters require to first set a selector
    }

    // translate ros parameter name to GenICam name

    try {
      const std::string & name = param.at(p[i].get_name());

      switch (p[i].get_type()) {
        case rclcpp::PARAMETER_BOOL:
          rcg::setBoolean(nodemap, name.c_str(), p[i].as_bool(), true);
          break;

        case rclcpp::PARAMETER_INTEGER:
          rcg::setInteger(nodemap, name.c_str(), p[i].as_int(), true);
          break;

        case rclcpp::PARAMETER_DOUBLE:
          rcg::setFloat(nodemap, name.c_str(), p[i].as_double(), true);
          break;

        case rclcpp::PARAMETER_STRING:
          {
            // if the parameter is an enum and the given value does not fit,
            // an exception is thrown that causes rejection of the value
            rcg::setString(nodemap, name.c_str(), p[i].as_string().c_str(), true);

            if (p[i].get_name() == "out1_mode") {
              remote_out1_mode = p[i].as_string();
            }
          }
          break;

        default:
          ret.successful = false;
          ret.reason = "Internal error: Unknown type of parameter " + p[i].get_name();
          break;
      }
    } catch (const std::out_of_range &) {
      ret.successful = false;
      ret.reason = "Internal error: unknown parameter " + p[i].get_name();
      break;
    } catch (const std::exception & ex) {
      ret.successful = false;
      ret.reason = "Cannot set parameter " + p[i].get_name() + ": " + ex.what();
      break;
    }
  }

  return ret;
}

void GenICamDriver::triggerDepthAcquisition(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<rc_common_msgs::srv::Trigger::Request>,
  std::shared_ptr<rc_common_msgs::srv::Trigger::Response> res)
{
  std::lock_guard<std::recursive_mutex> lock(param_mtx);

  if (nodemap) {
    std::string mode;
    get_parameter("depth_acquisition_mode", mode);

    if (mode != "Continuous") {
      try {
        RCLCPP_DEBUG(this->get_logger(), "Triggering stereo matching");

        rcg::callCommand(nodemap, "DepthAcquisitionTrigger", true);

        res->return_code.value = rc_common_msgs::msg::ReturnCodeConstants::SUCCESS;
        res->return_code.message = "Stereo matching was triggered.";
      } catch (const std::exception & ex) {
        res->return_code.value = rc_common_msgs::msg::ReturnCodeConstants::INTERNAL_ERROR;
        res->return_code.message = ex.what();
        RCLCPP_ERROR(this->get_logger(), ex.what());
      }
    } else {
      res->return_code.value = rc_common_msgs::msg::ReturnCodeConstants::NOT_APPLICABLE;
      res->return_code.message =
        "Triggering stereo matching is only possible if depth_acquisition_mode is set to SingleFrame "
        "or SingleFrameOut1!";
      RCLCPP_DEBUG(this->get_logger(), "%s", res->return_code.message.c_str());
    }
  } else {
    res->return_code.value = rc_common_msgs::msg::ReturnCodeConstants::NOT_APPLICABLE;
    res->return_code.message = "Not connected";
  }
}

void GenICamDriver::publishConnectionDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::recursive_mutex> lock(updater_mtx);

  stat.add("connection_loss_total", connection_loss_total);
  stat.add("complete_buffers_total", complete_buffers_total);
  stat.add("incomplete_buffers_total", incomplete_buffers_total);
  stat.add("image_receive_timeouts_total", image_receive_timeouts_total);
  stat.add("current_reconnect_trial", current_reconnect_trial);

  // general connection status

  if (device_serial.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Disconnected");
    return;
  }

  // at least we are connected to gev server

  stat.add("ip_interface", device_interface);
  stat.add("ip_address", device_ip);
  stat.add("gev_packet_size", gev_packet_size);

  if (scomponents) {
    if (streaming) {
      // someone subscribed to images, and we actually receive data via GigE vision
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");
    } else {
      // someone subscribed to images, but we do not receive any data via GigE vision (yet)
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No data");
    }
  } else {
    // no one requested images -> node is ok but stale
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Idle");
  }
}

void GenICamDriver::publishDeviceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::recursive_mutex> lock(updater_mtx);

  if (device_serial.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Info");
    stat.add("model", device_model);
    stat.add("image_version", device_version);
    stat.add("serial", device_serial);
    stat.add("mac", device_mac);
    stat.add("user_id", device_name);
  }
}

void GenICamDriver::process()
{
  try {
    RCLCPP_INFO(this->get_logger(), "Processing thread started");

    // loop until nodelet is killed

    while (running) {
      streaming = false;

      // report standard exceptions and try again

      try {
        std::shared_ptr<GenApi::CChunkAdapter> chunkadapter;

        configure(); // connect and setup parameters and publishers

        // enable chunk information and setup up adapter

        {
          std::lock_guard<std::recursive_mutex> lock(param_mtx);

          rcg::setBoolean(nodemap, "ChunkModeActive", true, true);
          chunkadapter = rcg::getChunkAdapter(nodemap, dev->getTLType());
        }

        // start streaming

        std::vector<std::shared_ptr<rcg::Stream>> stream = dev->getStreams();

        if (stream.size() == 0) {
          throw std::invalid_argument("Device does not offer streams");
        }

        stream[0]->open();
        stream[0]->startStreaming();

        current_reconnect_trial = 1;

        updater.force_update();

        RCLCPP_INFO(this->get_logger(), "Start streaming images");

        // grabbing and publishing

        while (running) {
          // grab next buffer

          const rcg::Buffer * buffer = stream[0]->grab(500);
          std::string out1_mode_on_sensor;

          // process buffer

          if (buffer) {
            streaming = true;

            if (buffer->getIsIncomplete()) {
              incomplete_buffers_total++;
              out1_mode_on_sensor = "";
            } else {
              complete_buffers_total++;

              std::lock_guard<std::recursive_mutex> lock(param_mtx);

              if (gev_packet_size == 0) {
                gev_packet_size = rcg::getInteger(nodemap, "GevSCPSPacketSize", 0, 0, true, false);
              }

              // attach buffer to nodemap to access chunk data

              chunkadapter->AttachBuffer(reinterpret_cast<std::uint8_t *>(buffer->getGlobalBase()),
                buffer->getSizeFilled());

              // get out1 mode on device, which may have changed

              rcg::setEnum(nodemap, "ChunkLineSelector", "Out1", true);
              out1_mode_on_sensor = rcg::getEnum(nodemap, "ChunkLineSource", true);

              // publish all parts of buffer

              uint32_t npart = buffer->getNumberOfParts();
              for (uint32_t part = 0; part < npart; part++) {
                if (buffer->getImagePresent(part)) {
                  uint64_t pixelformat = buffer->getPixelFormat(part);
                  for (auto && p : pub) {
                    p->publish(buffer, part, pixelformat);
                  }
                }
              }

              // detach buffer from nodemap

              chunkadapter->DetachBuffer();
            }
          } else {
            image_receive_timeouts_total++;
            streaming = false;

            // get out1 mode from sensor (this is also used to check if the
            // connection is still valid)

            std::lock_guard<std::recursive_mutex> lock(param_mtx);
            rcg::setEnum(nodemap, "LineSelector", "Out1", true);
            out1_mode_on_sensor = rcg::getString(nodemap, "LineSource", true, true);
          }

          {
            std::lock_guard<std::recursive_mutex> lock(param_mtx);

            if (update_exp_values) {
              update_exp_values = false;

              double exp = rcg::getFloat(nodemap, "ExposureTime", 0, 0, true, true);
              set_parameter(rclcpp::Parameter("camera_exp_value", exp));

              rcg::setEnum(nodemap, "GainSelector", "All", false);
              double gain = rcg::getFloat(nodemap, "Gain", 0, 0, true, true);
              set_parameter(rclcpp::Parameter("camera_gain_value", gain));
            }

            if (update_wb_values) {
              update_wb_values = false;

              if (rcg::setEnum(nodemap, "BalanceRatioSelector", "Red", false)) {
                double ratio = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, true, true);
                set_parameter(rclcpp::Parameter("camera_wb_red", ratio));
              }

              if (rcg::setEnum(nodemap, "BalanceRatioSelector", "Blue", false)) {
                double ratio = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, true, true);
                set_parameter(rclcpp::Parameter("camera_wb_blue", ratio));
              }
            }

            // update out1 mode, if it is different to current settings on sensor
            // (which is the only GEV parameter which could have changed outside this code,
            //  i.e. on the rc_visard by the stereomatching module)

            if (out1_mode_on_sensor.size() == 0) {
              // use current settings if the value on the sensor cannot be determined
              out1_mode_on_sensor = remote_out1_mode;
            }

            if (out1_mode_on_sensor != remote_out1_mode) {
              remote_out1_mode = out1_mode_on_sensor;
              set_parameter(rclcpp::Parameter("out1_mode", out1_mode_on_sensor));
            }
          }
        }

        // stop streaming

        stream[0]->stopStreaming();
        stream[0]->close();

        // stop publishing

        {
          std::lock_guard<std::recursive_mutex> lock(updater_mtx);

          device_model = "";
          device_version = "";
          device_serial = "";
          device_mac = "";
          device_name = "";
          device_interface = "";
          device_ip = "";
          gev_packet_size = 0;
          streaming = false;

          updater.force_update();
        }
      } catch (const NoDeviceException & ex) {
        // report error, wait and retry

        RCLCPP_WARN(this->get_logger(), ex.what());

        {
          std::lock_guard<std::recursive_mutex> lock(updater_mtx);

          current_reconnect_trial++;
          streaming = false;

          updater.force_update();
        }

        sleep(3);
      } catch (const std::exception & ex) {
        {
          std::lock_guard<std::recursive_mutex> lock(updater_mtx);

          // close everything and report error

          if (device_ip.size() > 0) {
            connection_loss_total++;
          }

          device_model = "";
          device_version = "";
          device_serial = "";
          device_mac = "";
          device_name = "";
          device_interface = "";
          device_ip = "";
          gev_packet_size = 0;
          streaming = false;

          current_reconnect_trial++;

          RCLCPP_ERROR(this->get_logger(), ex.what());

          updater.force_update();
        }

        sleep(3);
      }

      // undeclare parameters, remove publishers and close device

      cleanup();
    }
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(this->get_logger(), ex.what());
  } catch (...) {
    RCLCPP_FATAL(this->get_logger(), "Unknown exception");
  }

  {
    std::lock_guard<std::recursive_mutex> lock(updater_mtx);

    device_model = "";
    device_version = "";
    device_serial = "";
    device_mac = "";
    device_name = "";
    device_interface = "";
    device_ip = "";
    gev_packet_size = 0;
    streaming = false;

    updater.force_update();
  }

  running = false;
  RCLCPP_INFO(this->get_logger(), "Processing thread stopped");
}

}  // namespace rc

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rc::GenICamDriver)
