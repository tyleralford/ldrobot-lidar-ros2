//  Copyright 2024 Walter Lucetti
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
////////////////////////////////////////////////////////////////////////////////

#include "ldlidar_component.hpp"
#include "ldlidar_tools.hpp"
#include <algorithm>
#include <cctype>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rmw/types.h"
#include "rclcpp/parameter.hpp"
#include "rclcpp/exceptions.hpp"

using namespace std::placeholders;

namespace ldlidar
{

const int QOS_QUEUE_SIZE = 10;

static std::string lowercase(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

LdLidarComponent::LdLidarComponent(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("lidar_node", "", options), _diagUpdater(this)
{
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " LDRobot DToF Lidar Lifecycle Component ");
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "****************************************");
  RCLCPP_INFO(
    get_logger(),
    " + State: 'unconfigured [1]'. Use lifecycle commands "
    "to configure [1] or shutdown [5]");

  // ----> Diagnostic
  _diagUpdater.add("LDLidar Diagnostic", this, &LdLidarComponent::callback_updateDiagnostic);
  _diagUpdater.setHardwareID("ldlidar");  // replaced by the model once the parameters are read
  // <---- Diagnostic
}

LdLidarComponent::~LdLidarComponent()
{
}

bool LdLidarComponent::getParameters()
{
  RCLCPP_INFO_STREAM(get_logger(), "****** NODE PARAMETERS ******");

  // DEBUG parameters
  getDebugParams();

  // COMMUNICATION parameters
  getCommParams();

  // LIDAR parameters
  return getLidarParams();
}

void LdLidarComponent::getDebugParams()
{
  // ----> Debug mode initialization from parameters
  getParam("general.debug_mode", _debugMode, _debugMode, "Enable debug messages");
  if (_debugMode) {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting DEBUG level for logger");
    } else {
      RCLCPP_INFO(get_logger(), "*** Debug Mode enabled ***");
    }
  } else {
    rcutils_ret_t res = rcutils_logging_set_logger_level(
      get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

    if (res != RCUTILS_RET_OK) {
      RCLCPP_INFO(get_logger(), "Error setting INFO level for logger");
    }
  }

  RCLCPP_DEBUG(
    get_logger(), "[ROS2] Using RMW_IMPLEMENTATION = %s", rmw_get_implementation_identifier());
  // <---- Debug mode initialization from parameters
}

void LdLidarComponent::getCommParams()
{
  RCLCPP_INFO(get_logger(), "+++ COMMUNICATION PARAMETERS +++");

  // ----> Communication
  getParam(
    "comm.serial_port", _serialPort, _serialPort, "Communication serial port path", true,
    " * Serial port: ");

  getParam(
    "comm.baudrate", _baudrate, _baudrate, "Serial baudrate", false,
    " * Baudrate: ");
  getParam(
    "comm.timeout_msec", _readTimeOut_msec, _readTimeOut_msec, "Data reading timeout in msec",
    false, " * Timeout [msec]: ");

    // <---- Communication
}

bool LdLidarComponent::getLidarParams()
{
  RCLCPP_INFO(get_logger(), "+++ LIDAR PARAMETERS +++");
  nav2_util::LifecycleNode::integer_range limits_int;

  // ----> Lidar config
  getParam("lidar.model", _lidarModel, _lidarModel, " * Lidar frame: ", "Name of the lidar frame");
  if (_lidarModel == "LD06") {
    _lidarType = ldlidar::LDType::LD_06;
  } else if (_lidarModel == "LD19") {
    _lidarType = ldlidar::LDType::LD_19;
  } else if (_lidarModel == "STL27L") {
    _lidarType = ldlidar::LDType::STL_27L;
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(), " !!! The parameter 'lidar.model' is not valid! -> " << _lidarModel.c_str() );
    RCLCPP_ERROR_STREAM(
      get_logger(), "     * Valid values: 'LD06', 'LD19', 'STL27L'");
    return false;
  }

  getParam(
    "lidar.frame_id", _frameId, _frameId, "Name of the lidar frame", true,
    " * Lidar frame: ");

  std::string units_str = "M";
  getParam("lidar.units", units_str, units_str, "Measure units", true, " * Units: ");

  if (units_str == "MM") {
    _distScale = 1.0f;
  } else if (units_str == "CM") {
    _distScale = 0.1f;
  } else {
    if (units_str != "M") {
      RCLCPP_WARN(get_logger(), "Parameter 'lidar.units' not valid, using the default value: 'M'");
    }
    _distScale = 0.001f;
  }

  std::string rot_verse = "CCW";
  getParam(
    "lidar.rot_verse", rot_verse, rot_verse, " * Rotation verse: ",
    "Rotation verse, use 'CW' if upsidedown");
  if (rot_verse == "CW") {
    _counterclockwise = false;
  } else {
    if (rot_verse != "CCW") {
      RCLCPP_WARN(
        get_logger(), "Parameter 'lidar.rot_verse' not valid, using the default value: 'CCW'");
    }
    _counterclockwise = true;
  }

  getParam(
    "lidar.bins", _bins, _bins, "Fixed number of data beams. 0 for dynamic", true,
    " * Bins: ");
  getParam(
    "lidar.stamp_latency_s", _stampLatency, _stampLatency,
    "Serial + host read latency subtracted from the scan stamp", true,
    " * Stamp latency [s]: ");
  _diagUpdater.setHardwareID(lowercase(_lidarModel));
  getParam(
    "lidar.range_min", _rangeMin, _rangeMin, "Minimum distance in units", false,
    " * Min. distance: ");
  getParam(
    "lidar.range_max", _rangeMax, _rangeMax, "Maximum distance in units", false,
    " * Max. distance: ");
  getParam(
    "lidar.enable_angle_crop", _enableAngleCrop, _enableAngleCrop, "Angle cropping", false,
    " * Angle cropping: ");
  getParam(
    "lidar.angle_crop_min", _angleCropMin, _angleCropMin, "Angle cropping minimum angle",
    false, " * Angle cropping min angle: ");
  getParam(
    "lidar.angle_crop_max", _angleCropMax, _angleCropMax, "Angle cropping maximum angle",
    false, " * Angle cropping max angle: ");
  // <---- Lidar config

  return true;
}

nav2_util::CallbackReturn LdLidarComponent::on_configure(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_configure: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                     << "] -> Inactive");

  // Initialize parameters from yaml file
  if (!getParameters()) {
    return nav2_util::CallbackReturn::ERROR;
  }

  // ----> Initialize topics
  _scanTopic = _topicRoot + std::string("scan");
  // <---- Initialize topics

  // ----> Initialize publisher
  rclcpp::QoS qos(QOS_QUEUE_SIZE);

  auto pub_opt = rclcpp::PublisherOptions();
  pub_opt.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

  if (!_scanPub) {
    _scanPub = this->create_publisher<sensor_msgs::msg::LaserScan>(_scanTopic, qos, pub_opt);
  }
  // <---- Initialize publisher

  // ----> Connect to Lidar
  if (!initLidar()) {
    return nav2_util::CallbackReturn::ERROR;  // Transition to Finalized state
    // Note: we could use FAILURE instead of ERROR to remain in unconfigured
    // state and try again to connect
  }
  // <---- Connect to Lidar

  RCLCPP_INFO(
    get_logger(),
    " + State: 'inactive [2]'. Use lifecycle commands to "
    "activate [3], cleanup [2] or shutdown "
    "[6]");
  return nav2_util::CallbackReturn::SUCCESS;
}  // namespace ldlidar

nav2_util::CallbackReturn LdLidarComponent::on_activate(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_activate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                  << "] -> Active");

  // create bond connection
  createBond();

  // Activate publisher
  _scanPub->on_activate();

  // Reset per-activation scan state
  _firstScan = true;
  _pubFreq = 0.0;
  _pubStamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Start lidar thread
  startLidarThread();

  RCLCPP_INFO(
    get_logger(),
    " + State: 'active [3]'. Use lifecycle commands to "
    "deactivate [4] or shutdown [7]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_deactivate(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "on_deactivate: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                      << "] -> Inactive");

  // destroy bond connection
  destroyBond();

  // Deactivate publisher
  _scanPub->on_deactivate();

  // Stop lidar thread then close the driver (order matters)
  stopLidarThread();
  _lidar->Stop();

  RCLCPP_INFO(
    get_logger(),
    " + State: 'inactive [2]'. Use lifecycle commands to "
    "activate [3], cleanup [2] or shutdown "
    "[6]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_cleanup(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_cleanup: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                 << "] -> Unconfigured");

  _scanPub.reset();
  _lidar.reset();

  RCLCPP_INFO(
    get_logger(),
    " + State: 'unconfigured [1]'. Use lifecycle commands "
    "to configure [1] or shutdown [5]");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_shutdown(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_shutdown: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                                  << "] -> Finalized");

  _scanPub.reset();

  RCLCPP_INFO_STREAM(get_logger(), " + State: 'finalized [4]'. Press Ctrl+C to kill...");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn LdLidarComponent::on_error(const lc::State & prev_state)
{
  RCLCPP_DEBUG_STREAM(
    get_logger(), "on_error: " << prev_state.label() << " [" << static_cast<int>(prev_state.id())
                               << "] -> Finalized");

  RCLCPP_INFO_STREAM(get_logger(), " + State: 'finalized [4]'. Press Ctrl+C to kill...");
  return nav2_util::CallbackReturn::FAILURE;
}

void LdLidarComponent::publishLaserScan(ldlidar::Points2D & src, double lidar_spin_freq)
{
  // The SDK drops the first revolution after a (re)start; so do we, to be safe.
  if (_firstScan) {
    _firstScan = false;
    return;
  }

  std::vector<RawPoint> raw;
  raw.reserve(src.size());
  for (const auto & p : src) {
    raw.push_back({p.angle, p.distance, p.intensity, p.stamp});
  }

  BinningParams prm;
  prm.bins = _bins;
  prm.counterclockwise = _counterclockwise;
  prm.dist_scale = _distScale;
  prm.range_min = _rangeMin;
  prm.range_max = _rangeMax;
  prm.crop = _enableAngleCrop;
  prm.crop_min_deg = _angleCropMin;
  prm.crop_max_deg = _angleCropMax;
  prm.stamp_latency_s = _stampLatency;

  BinnedScan binned;
  if (!bin_scan(raw, lidar_spin_freq, prm, binned)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Scan dropped: no spin rate or no usable point stamps");
    return;
  }

  const rclcpp::Time stamp(static_cast<int64_t>(binned.stamp_s * 1e9), get_clock()->get_clock_type());
  if (_pubStamp.nanoseconds() != 0 && stamp > _pubStamp) {
    _pubFreq = 1.0 / (stamp - _pubStamp).seconds();
  }
  _pubStamp = stamp;
  _lastPoints = static_cast<int>(src.size());

  auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  msg->header.stamp = stamp;
  msg->header.frame_id = _frameId;
  msg->angle_min = binned.angle_min;
  msg->angle_increment = binned.angle_increment;
  msg->angle_max = binned.angle_min + binned.angle_increment * (binned.ranges.size() - 1);
  msg->time_increment = binned.time_increment;
  msg->scan_time = binned.scan_time;
  msg->range_min = _rangeMin;
  msg->range_max = _rangeMax;
  msg->ranges = std::move(binned.ranges);
  msg->intensities = std::move(binned.intensities);
  _scanPub->publish(std::move(msg));
}

bool LdLidarComponent::initLidar()
{
  _lidar = std::make_unique<ldlidar::LDLidarDriver>();

  // Stamp packets on the node's clock so they follow use_sim_time.
  _lidar->RegisterGetTimestampFunctional(
    [this]() {return static_cast<uint64_t>(this->now().nanoseconds());});
  _lidar->EnableFilterAlgorithnmProcess(true);

  if (!initLidarComm()) {
    return false;
  }

  return true;
}

bool LdLidarComponent::initLidarComm()
{
  if (_lidar->Start(_lidarType, _serialPort, _baudrate, ldlidar::COMM_SERIAL_MODE)) {
    RCLCPP_INFO_STREAM(get_logger(), "***** LDLidar opened on port '" << _serialPort << "' *****");
  } else {
    RCLCPP_ERROR(get_logger(), "!!! LDLidar not opened !!!");
    return false;
  }

  if (_lidar->WaitLidarCommConnect(3000)) {
    RCLCPP_INFO(get_logger(), " * LDLidar communication OK");
  } else {
    RCLCPP_ERROR(get_logger(), " !!! LDLidar communication KO !!!");
    return false;
  }

  return true;
}

void LdLidarComponent::startLidarThread()
{
  _lidarThread = std::thread(&LdLidarComponent::lidarThreadFunc, this);
}

void LdLidarComponent::stopLidarThread()
{
  if (!_threadStop) {
    _threadStop = true;
    RCLCPP_DEBUG(get_logger(), "Stopping lidar thread...");
    try {
      if (_lidarThread.joinable()) {
        _lidarThread.join();
      }
    } catch (std::system_error & e) {
      RCLCPP_WARN(get_logger(), "Lidar thread joining exception: %s", e.what());
    }
  }
}

void LdLidarComponent::lidarThreadFunc()
{
  RCLCPP_DEBUG(get_logger(), "Lidar thread started");

  _threadStop = false;
  _publishing = false;

  ldlidar::Points2D laser_scan_points;
  double lidar_scan_freq = 0.0;

  while (1) {
    // ----> Interruption check
    if (!rclcpp::ok()) {
      RCLCPP_DEBUG(get_logger(), "Ctrl+C received: stopping grab thread");
      _threadStop = true;
    }

    if (_threadStop) {
      RCLCPP_DEBUG(get_logger(), "Lidar thread stopped");
      break;
    }
    // <---- Interruption check

    // The publisher's own count: count_subscribers(name) expands the name but not a remap.
    int nSub = _scanPub->get_subscription_count();
    bool published = false;
    if (nSub > 0) {
      _publishing = true;
      switch (_lidar->GetLaserScanData(laser_scan_points, _readTimeOut_msec)) {
        case ldlidar::LidarStatus::NORMAL:
          _lidar->GetLidarScanFreq(lidar_scan_freq);
          publishLaserScan(laser_scan_points, lidar_scan_freq);
          published = true;
          break;
        case ldlidar::LidarStatus::DATA_TIME_OUT:
          RCLCPP_ERROR(
            get_logger(), "get ldlidar data is time out, please check your lidar device.");
          break;
        case ldlidar::LidarStatus::DATA_WAIT:
          break;
        default:
          break;
      }
    } else {
      _publishing = false;
    }

    // A scan is ready once a turn, and the SDK's read does not wait for it. After publishing one,
    // wait most of a turn; then poll, so the next goes out within milliseconds of being complete
    // (sleeping a whole turn every pass published it up to a turn late).
    using std::chrono::milliseconds;
    using std::chrono::nanoseconds;
    if (published && _lidar->GetLidarScanFreq(lidar_scan_freq) && lidar_scan_freq != 0.0) {
      rclcpp::sleep_for(nanoseconds(int64_t(0.8e9 / lidar_scan_freq)));
    } else if (nSub > 0) {
      rclcpp::sleep_for(milliseconds(2));
    } else {
      rclcpp::sleep_for(milliseconds(100));
    }
  }

  RCLCPP_DEBUG(get_logger(), "Lidar thread finished");
}

void LdLidarComponent::callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  rclcpp_lifecycle::State state = get_current_state();

  if (state.id() == 3) {  // ACTIVE
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK, std::string(
        "Node state: ") + state.label());

    double spin_hz;
    _lidar->GetLidarScanFreq(spin_hz);
    stat.addf("Single measure", "%.3f Hz", spin_hz);
    stat.addf("Points per scan", "%d", _lastPoints.load());

    if (_publishing) {
      stat.addf("Publishing", "%.3f Hz", _pubFreq);
    } else {
      stat.add("Publishing", "NO SUBSCRIBERS");
    }
  } else if (state.id() == 1 || state.id() == 2) {  // UNCONFIGURED || INACTIVE
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(
        "Node state: ") + state.label());
  } else {  // SHUTDOWN || ERROR
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string(
        "Node state: ") + state.label());
  }
}

}  // namespace ldlidar

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(ldlidar::LdLidarComponent)
