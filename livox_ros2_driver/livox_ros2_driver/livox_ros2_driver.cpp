//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <chrono>
#include <csignal>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include "include/livox_ros2_driver.h"

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"


#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"
#include "livox_interfaces/msg/custom_point.h"
#include "livox_interfaces/msg/custom_msg.h"

namespace
{
  const int32_t kSdkVersionMajorLimit = 2;

  inline void SignalHandler(int signum)
  {
    rclcpp::shutdown();
    exit(signum);
  }
}

namespace livox_ros
{
LivoxDriver::LivoxDriver(const rclcpp::NodeOptions & node_options)
: Node("livox_driver_node", node_options)
{
  RCLCPP_INFO(this->get_logger(), "Livox Ros Driver Version: %s",
    LIVOX_ROS_DRIVER_VERSION_STRING);

  signal(SIGINT, SignalHandler);

  /** Check sdk version */
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit) {
    RCLCPP_INFO(this->get_logger(),
      "The SDK version[%d.%d.%d] is too low", _sdkversion.major,
      _sdkversion.minor, _sdkversion.patch);
    rclcpp::shutdown();
    return;
  }

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id;

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", data_src);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", output_type);
  this->declare_parameter("frame_id", "frame_default");
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);
  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.1) {
    publish_freq = 0.1;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ =
    std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  int ret = 0;
  if (data_src == kSourceRawLidar) {
    RCLCPP_INFO(this->get_logger(), "Data Source is raw lidar.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    RCLCPP_INFO(this->get_logger(), "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsLidar *read_lidar = LdsLidar::GetInstance(1000 / publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));
    ret = read_lidar->InitLdsLidar(bd_code_list, user_config_path.c_str());
    if (!ret) {
      RCLCPP_INFO(this->get_logger(), "Init lds lidar success!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Init lds lidar fail!");
    }
    // Optional: time sync status of each lidar on <namespace>/diagnostics, from the status in its point packets
    if (this->declare_parameter("publish_diagnostics", false)) {
      diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
      diagnostics_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                                   std::bind(&LivoxDriver::publishDiagnostics, this));
    }
  } else if (data_src == kSourceRawHub) {
    RCLCPP_INFO(this->get_logger(), "Data Source is hub.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    RCLCPP_INFO(this->get_logger(), "Config file : %s",
        user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsHub *read_hub = LdsHub::GetInstance(1000 / publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_hub));
    ret = read_hub->InitLdsHub(bd_code_list, user_config_path.c_str());
    if (!ret) {
      RCLCPP_INFO(this->get_logger(), "Init lds hub success!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Init lds hub fail!");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Data Source is lvx file.");

    std::string cmdline_file_path;
    this->get_parameter("cmdline_file_path", cmdline_file_path);

    do {
      if (!IsFilePathValid(cmdline_file_path.c_str())) {
        RCLCPP_INFO(this->get_logger(), "File path invalid : %s !", cmdline_file_path.c_str());
        break;
      }

      std::string rosbag_file_path;
      int path_end_pos = cmdline_file_path.find_last_of('.');
      rosbag_file_path = cmdline_file_path.substr(0, path_end_pos);
      rosbag_file_path += ".bag";

      LdsLvx *read_lvx = LdsLvx::GetInstance(1000 / publish_freq);
      lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lvx));
      lddc_ptr_->CreateBagFile(rosbag_file_path);
      int ret = read_lvx->InitLdsLvx(cmdline_file_path.c_str());
      if (!ret) {
        RCLCPP_INFO(this->get_logger(), "Init lds lvx file success!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Init lds lvx file fail!");
      }
    } while (0);
  }

  poll_thread_ = std::make_shared<std::thread>(&LivoxDriver::pollThread, this);
}

LivoxDriver::~LivoxDriver()
{
  exit_signal_.set_value();
  poll_thread_->join();
}

void LivoxDriver::publishDiagnostics()
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  LdsLidar *lds = static_cast<LdsLidar *>(lddc_ptr_->lds_);
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = this->now();
  auto now = std::chrono::steady_clock::now();
  for (uint8_t i = 0; i < kMaxLidarCount; ++i) {
    const LidarDevice &lidar = lds->lidars_[i];
    if (lidar.info.broadcast_code[0] == '\0') {
      continue;  // unused handle
    }
    DiagnosticStatus status;
    status.name = std::string("livox_driver: ") + lidar.info.broadcast_code + " time sync";
    status.hardware_id = lidar.info.broadcast_code;
    double age = std::chrono::duration<double>(now - lds->last_error_time_[i]).count();
    const LidarErrorCode &code = lds->last_error_[i].lidar_error_code;
    const uint8_t timestamp_type = lds->last_timestamp_type_[i];
    if (lidar.connect_state != kConnectStateSampling) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Lidar not sampling (disconnected?)";
    } else if (age > 3.0) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "No point packets from the lidar";
    } else if (timestamp_type == kTimestampTypePpsGps) {
      status.level = DiagnosticStatus::OK;
      status.message = "GPS sync (PPS + UTC)";
    } else if (timestamp_type == kTimestampTypePps) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "PPS only, no UTC (timesync / GPRMC missing?)";
    } else if (timestamp_type == kTimestampTypeNoSync) {
      status.level = DiagnosticStatus::ERROR;
      status.message = "No sync (PPS missing?)";
    } else {
      status.level = DiagnosticStatus::ERROR;
      status.message = "Unexpected timestamp type " + std::to_string(timestamp_type);
    }
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "timestamp_type";  // 0 no sync, 1 PTP, 3 GPS (PPS + UTC), 4 PPS
    kv.value = std::to_string(timestamp_type);
    status.values.push_back(kv);
    kv.key = "time_sync_status";
    kv.value = std::to_string(code.time_sync_status);
    status.values.push_back(kv);
    kv.key = "pps_status";  // 1: PPS signal OK
    kv.value = std::to_string(code.device_status);
    status.values.push_back(kv);
    kv.key = "error_code";
    kv.value = std::to_string(lds->last_error_[i].error_code);
    status.values.push_back(kv);
    msg.status.push_back(status);
  }
  diagnostics_pub_->publish(msg);
}

void LivoxDriver::pollThread()
{
  std::future_status status;

  do {
    lddc_ptr_->DistributeLidarData();
    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}
}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::LivoxDriver)
