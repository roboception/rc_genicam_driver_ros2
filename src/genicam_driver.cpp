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
  device_descr.dynamic_typing = true; // Without this, undeclare_parameter() throws in Galactic onward.

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
  std::vector<std::shared_ptr<rcg::System>> systems = rcg::System::getSystems();
  std::vector<std::shared_ptr<rcg::Device>> ret;

  for (const auto &system: systems) {
    system->open();

    std::vector<std::shared_ptr<rcg::Interface>> interfaces = system->getInterfaces();

    for (const auto &interf: interfaces) {
      if (interf->getTLType() == "GEV" &&
        (iname.size() == 0 ||
        std::find(iname.begin(), iname.end(), interf->getID()) != iname.end()))
      {
        interf->open();

        std::vector<std::shared_ptr<rcg::Device>> devices = interf->getDevices();

        for (const auto &device: devices) {
          if ((device->getVendor() == "Roboception GmbH" ||
            device->getModel().substr(0,
            9) == "rc_visard" || device->getModel().substr(0, 7) == "rc_cube") &&
            (devid == "*" || device->getID() == devid || device->getSerialNumber() == devid ||
            device->getDisplayName() == devid))
          {
            ret.push_back(device);
          }
        }

        interf->close();
      }
    }

    system->close();
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
  const std::shared_ptr<GenApi::CNodeMapRef> & nodemap, const std::string & name,
  const char *description, double float_scale)
{
  bool ret = false;

  try {
    std::lock_guard<std::recursive_mutex> lock(param_mtx);

    GenApi::INode * node = nodemap->_GetNode(name.c_str());

    if (node != 0) {
      if (GenApi::IsReadable(node) && GenApi::IsWritable(node)) {
        rcl_interfaces::msg::ParameterDescriptor param_descr;
        param_descr.dynamic_typing = true; // Without this, undeclare_parameter() throws in Galactic onward.

        if (description)
        {
          param_descr.description = description;
        }
        else
        {
          param_descr.description = node->GetDescription();
        }

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

              float_range.from_value = std::round(1e6*p->GetMin()/float_scale)/1e6;
              float_range.to_value = std::round(1e6*p->GetMax()/float_scale)/1e6;

              float_range.step = 0;
              if (p->GetIncMode() == GenApi::fixedIncrement) {
                float_range.step = p->GetInc()/float_scale;
              }

              param_descr.floating_point_range.push_back(float_range);

              param[ros_name] = name;
              param_float_scale[ros_name] = float_scale;

              double value=p->GetValue(false, false)/float_scale;
              value=std::max(value, float_range.from_value);
              value=std::min(value, float_range.to_value);

              declare_parameter(ros_name, value, param_descr);
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
            RCLCPP_WARN_STREAM(
              this->get_logger(),
              "Parameter has unsupported type: " << ros_name << " (" << name << ")");
            break;
        }
      } else {
        RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Parameter not readable or writable: " << ros_name << " (" << name << ")");
      }
    } else {
      RCLCPP_WARN_STREAM(
        this->get_logger(), "Parameter does not exist (old firmware?): " << ros_name << " (" << name << ")");
    }
  } catch (const GENICAM_NAMESPACE::GenericException & ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Parameter: " << ros_name << " (" << name << ")");
  }

  return ret;
}

bool GenICamDriver::declareGenICamParameter(
  const std::string & ros_name,
  const std::shared_ptr<GenApi::CNodeMapRef> & nodemap, const std::string & name,
  const std::string & selector_name, const std::string & selector_value,
  const char *description, double float_scale)
{
  std::lock_guard<std::recursive_mutex> lock(param_mtx);
  bool ret = false;

  // set selector as requested

  std::string v = rcg::getEnum(nodemap, selector_name.c_str(), false);

  if (v == selector_value || rcg::setEnum(nodemap, selector_name.c_str(), selector_value.c_str(),
    false))
  {
    param_selector[ros_name] = std::make_pair(selector_name, selector_value);

    // declare parameter

    ret = declareGenICamParameter(ros_name, nodemap, name, description, float_scale);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Selector of parameter cannot be found or changed: " <<
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

  iocontrol_avail = nodemap->_GetNode("LineSource")->GetAccessMode() == GenApi::RW;

  // initialise variables for caching some values

  remote_out1_mode = "";
  update_exp_values = false;
  update_wb_values = false;

  // Represent GenICam parameter ExposureAuto with ROS parameters
  // camera_exp_control and camera_exp_auto_mode as in RestAPI interface
  // of Roboception devices

  {
    std::lock_guard<std::recursive_mutex> lock(param_mtx);

    std::string exp_auto=rcg::getString(nodemap, "ExposureAuto", false);

    rcl_interfaces::msg::ParameterDescriptor param_descr;
    param_descr.description = "Exposure control mode: [Manual, Auto, HDR]";
    param_descr.additional_constraints = "Manual|Auto|HDR";
    param_descr.dynamic_typing = true; // Without this, undeclare_parameter() throws in Galactic onward.

    std::string val="Auto";
    if (exp_auto == "Off") val="Manual";
    if (exp_auto == "HDR") val="HDR";

    declare_parameter("camera_exp_control", val, param_descr);

    param_descr.description = "Auto-exposure mode: [Normal, Out1High, AdaptiveOut1]";
    param_descr.additional_constraints = "Normal|Out1High|AdaptiveOut1";

    val="Normal";
    if (exp_auto == "Out1High") val="Out1High";
    if (exp_auto == "AdaptiveOut1") val="AdaptiveOut1";

    declare_parameter("camera_exp_auto_mode", val, param_descr);
  }

  // register parameter callback

  param_cb = add_on_set_parameters_callback(std::bind(&GenICamDriver::paramCallback, this, _1));

  // declare parameters and mapping to GenICam (all are allowed to fail if
  // parameters are not available in the given nodemap)

  // TODO: This could be made more generic by loading the mapping between ROS
  // and GenICam parameters from a configuration file instead of hard coding
  // it.

  declareGenICamParameter("camera_fps", nodemap, "AcquisitionFrameRate");
  declareGenICamParameter("camera_exp_max", nodemap, "ExposureTimeAutoMax", "Maximum exposure time in seconds", 1000000);
  declareGenICamParameter("camera_exp_auto_average_max", nodemap, "RcExposureAutoAverageMax");
  declareGenICamParameter("camera_exp_auto_average_min", nodemap, "RcExposureAutoAverageMin");
  declareGenICamParameter("camera_exp_width", nodemap, "ExposureRegionWidth");
  declareGenICamParameter("camera_exp_height", nodemap, "ExposureRegionHeight");
  declareGenICamParameter("camera_exp_offset_x", nodemap, "ExposureRegionOffsetX");
  declareGenICamParameter("camera_exp_offset_y", nodemap, "ExposureRegionOffsetY");
  declareGenICamParameter("camera_exp_value", nodemap, "ExposureTime", "Exposure time in seconds", 1000000);
  declareGenICamParameter("camera_gain_value", nodemap, "Gain", "GainSelector", "All");
  declareGenICamParameter("camera_gamma", nodemap, "Gamma");

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
  declareGenICamParameter("depth_exposure_adapt_timeout", nodemap, "DepthExposureAdaptTimeout");

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
  const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::recursive_mutex> lock(param_mtx);
  rcl_interfaces::msg::SetParametersResult ret;
  ret.successful = true;

  for (const auto &p: params) {
    // signal processing thread to update exposure and gain values if automatic
    // has been turned off

    if (p.get_name() == "camera_exp_control" &&
        (p.as_string() == "Manual" || p.as_string() == "HDR")) {
      update_exp_values = true;
    }

    // translate camera_exp_control and camera_exp_auto_mode parameters into
    // GenICam parameter ExposureAuto

    if (p.get_name() == "camera_exp_control" || p.get_name() == "camera_exp_auto_mode") {
      std::string exp_control, exp_auto_mode;

      if (p.get_name() == "camera_exp_control") {
        exp_control=p.as_string();
      } else {
        exp_control=get_parameter(std::string("camera_exp_control")).as_string();
      }

      if (p.get_name() == "camera_exp_auto_mode") {
        exp_auto_mode=p.as_string();
      } else {
        exp_auto_mode=get_parameter(std::string("camera_exp_auto_mode")).as_string();
      }

      std::string val="Auto";
      if (exp_control == "Manual") {
        val="Off";
      } else if (exp_control == "HDR") {
        val="HDR";
      } else if (exp_control == "Auto") {
        if (exp_auto_mode == "Normal") {
          val="Continuous";
        } else if (exp_auto_mode == "Out1High" || exp_auto_mode == "AdaptiveOut1") {
          val=exp_auto_mode;
        } else {
          RCLCPP_WARN_STREAM(this->get_logger(), "Ignoring illegal value '" << exp_auto_mode <<
            "' of parameter camera_exp_auto_mode");
          continue;
        }
      } else {
        RCLCPP_WARN_STREAM(this->get_logger(), "Ignoring illegal value '" << exp_control <<
          "' of parameter camera_exp_control");
        continue;
      }

      try
      {
        rcg::setEnum(nodemap, "ExposureAuto", val.c_str(), true);
      } catch (const std::exception & ex) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Cannot set parameter ExposureAuto to '" << val <<
          "'. Check if device supports this value: " << ex.what());
      }

      continue;
    }

    // signal processing thread to update white balancing values if automatic
    // has been turned off

    if (p.get_name() == "camera_wb_auto" && p.get_type() == rclcpp::PARAMETER_STRING &&
      p.as_string() == "Off") {
      update_wb_values = true;
    }

    // skip if value is cached one

    if (p.get_name() == "out1_mode" && p.get_type() == rclcpp::PARAMETER_STRING &&
      p.as_string() == remote_out1_mode) {
      continue;
    }

    // set selector if any

    try {
      const std::pair<std::string, std::string> & sel = param_selector.at(p.get_name());

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
      const std::string & name = param.at(p.get_name());

      switch (p.get_type()) {
        case rclcpp::PARAMETER_BOOL:
          rcg::setBoolean(nodemap, name.c_str(), p.as_bool(), true);
          break;

        case rclcpp::PARAMETER_INTEGER:
          rcg::setInteger(nodemap, name.c_str(), p.as_int(), true);
          break;

        case rclcpp::PARAMETER_DOUBLE:
          {
            double scale = param_float_scale.at(p.get_name());
            rcg::setFloat(nodemap, name.c_str(), scale*p.as_double(), true);
          }
          break;

        case rclcpp::PARAMETER_STRING:
          {
            // if the parameter is an enum and the given value does not fit,
            // an exception is thrown that causes rejection of the value
            rcg::setString(nodemap, name.c_str(), p.as_string().c_str(), true);

            if (p.get_name() == "out1_mode") {
              remote_out1_mode = p.as_string();
            }
          }
          break;

        default:
          ret.successful = false;
          ret.reason = "Internal error: Unknown type of parameter " + p.get_name();
          break;
      }
    } catch (const std::out_of_range &) {
      // permitted as this callback may also be called by parameters that we do
      // not manage ourselves
    } catch (const std::exception & ex) {
      ret.successful = false;
      ret.reason = "Cannot set parameter " + p.get_name() + ": " + ex.what();
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
              exp/=param_float_scale["camera_exp_value"];
              set_parameter(rclcpp::Parameter("camera_exp_value", exp));

              rcg::setEnum(nodemap, "GainSelector", "All", false);
              double gain = rcg::getFloat(nodemap, "Gain", 0, 0, true, true);
              set_parameter(rclcpp::Parameter("camera_gain_value", gain));
            }

            if (update_wb_values) {
              update_wb_values = false;

              if (rcg::setEnum(nodemap, "BalanceRatioSelector", "Red", false)) {
                double ratio = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, true, true);
                set_parameter(rclcpp::Parameter("camera_wb_ratio_red", ratio));
              }

              if (rcg::setEnum(nodemap, "BalanceRatioSelector", "Blue", false)) {
                double ratio = rcg::getFloat(nodemap, "BalanceRatio", 0, 0, true, true);
                set_parameter(rclcpp::Parameter("camera_wb_ratio_blue", ratio));
              }
            }

            // update out1 mode, if it is different to current settings on sensor
            // (which is the only GEV parameter which could have changed outside this code,
            //  i.e. on the rc_visard by the stereomatching module)

            if (out1_mode_on_sensor.size() == 0) {
              // use current settings if the value on the sensor cannot be determined
              out1_mode_on_sensor = remote_out1_mode;
            }

            if (iocontrol_avail && out1_mode_on_sensor != remote_out1_mode) {
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
