#include "robomaster_tether/hardware_interface.hpp"

#include <cmath>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robomaster_tether {

namespace {
auto get_logger() {
  return rclcpp::get_logger("hardware_interface");
}

constexpr const char *kWheelJointNames[4] = {
    "front_right_wheel_joint", "front_left_wheel_joint",
    "rear_right_wheel_joint", "rear_left_wheel_joint"};

// rad/s -> rpm, and clamps to the SDK's documented range
// (chassis wheel: int:[-1000, 1000], see protocol_api.html).
int rad_per_sec_to_rpm(double rad_per_sec) {
  const double rpm = rad_per_sec * 60.0 / (2.0 * M_PI);
  if (rpm > 1000.0) {
    return 1000;
  }
  if (rpm < -1000.0) {
    return -1000;
  }
  return static_cast<int>(rpm);
}
} // namespace

hardware_interface::CallbackReturn HardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Required: fail loudly here rather than time out against 0.0.0.0 later.
  if (info_.hardware_parameters.count("robot_ip")) {
    robot_ip_ = info_.hardware_parameters.at("robot_ip");
  }
  if (robot_ip_.empty()) {
    RCLCPP_ERROR(get_logger(),
                 "robot_ip is unset. It comes from ROBOMASTER_IP via "
                 "bringup.launch.py - set it in .env (direct-connect AP mode "
                 "is usually 192.168.2.1).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional overrides from the URDF <ros2_control><hardware><param> block,
  // falling back to the direct-connection defaults if not set.
  if (info_.hardware_parameters.count("enable_video")) {
    enable_video_ = info_.hardware_parameters.at("enable_video") == "true";
  }
  if (info_.hardware_parameters.count("control_port")) {
    control_port_ = std::stoi(info_.hardware_parameters.at("control_port"));
  }
  if (info_.hardware_parameters.count("wheel_radius_m")) {
    wheel_radius_m_ = std::stod(info_.hardware_parameters.at("wheel_radius_m"));
  }

  if (info_.joints.size() != kNumWheels) {
    RCLCPP_ERROR(get_logger(),
                 "expected exactly %zu wheel joints in URDF, got %zu",
                 kNumWheels, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_velocity_command_.fill(0.0);
  wheel_velocity_state_.fill(0.0);

  RCLCPP_INFO(get_logger(),
              "initialized for robot_ip=%s control_port=%d wheel_radius_m=%.4f",
              robot_ip_.c_str(), control_port_, wheel_radius_m_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < kNumWheels; ++i) {
    state_interfaces.emplace_back(kWheelJointNames[i],
                                  hardware_interface::HW_IF_VELOCITY,
                                  &wheel_velocity_state_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < kNumWheels; ++i) {
    command_interfaces.emplace_back(kWheelJointNames[i],
                                    hardware_interface::HW_IF_VELOCITY,
                                    &wheel_velocity_command_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn HardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  tcp_client_ = std::make_unique<TcpClient>();
  if (!tcp_client_->connect(robot_ip_, control_port_)) {
    RCLCPP_ERROR(
        get_logger(),
        "failed to connect/enter SDK mode at %s:%d - is the robot powered on, "
        "in direct-connection mode, and are you joined to its Wi-Fi hotspot?",
        robot_ip_.c_str(), control_port_);
    tcp_client_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Free mode: yaw axes of gimbal and chassis move independently. This
  // driver only ever commands the chassis, so gimbal-follows-chassis
  // coupling is irrelevant here but "free" is the least surprising
  // default. See protocol_api.html#robot-movement-mode-control.
  std::string response;
  tcp_client_->send_command("robot mode free", response);

  // Arm the camera from here because this interface owns the control port for
  // the whole session; camera_node.py can then read the video port without
  // needing a second client. Non-fatal: a robot with no camera should still
  // drive.
  if (enable_video_) {
    if (!tcp_client_->send_command("stream on", response) || response != "ok") {
      RCLCPP_WARN(get_logger(),
                  "'stream on' failed (got '%s') - driving is unaffected, but "
                  "camera_node will find no video. Set enable_video false to "
                  "skip this.",
                  response.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "video stream armed on port 40921.");
    }
  }

  wheel_velocity_command_.fill(0.0);
  wheel_velocity_state_.fill(0.0);

  start_sdk_bridge();

  RCLCPP_INFO(get_logger(), "activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void HardwareInterface::start_sdk_bridge() {
  stop_sdk_bridge();
  sdk_node_ = rclcpp::Node::make_shared("robomaster_sdk_bridge");
  sdk_bridge_ = std::make_unique<SdkBridge>(sdk_node_, tcp_client_.get());
  sdk_executor_ =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sdk_executor_->add_node(sdk_node_);
  sdk_spin_thread_ = std::thread([this]() {
    if (sdk_executor_) {
      sdk_executor_->spin();
    }
  });
}

void HardwareInterface::stop_sdk_bridge() {
  if (sdk_executor_) {
    sdk_executor_->cancel();
  }
  if (sdk_spin_thread_.joinable()) {
    sdk_spin_thread_.join();
  }
  sdk_bridge_.reset();
  sdk_executor_.reset();
  sdk_node_.reset();
}

hardware_interface::CallbackReturn HardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  stop_sdk_bridge();
  if (tcp_client_) {
    // Stop the chassis before dropping the connection - don't leave
    // it coasting on whatever the last command was.
    tcp_client_->send_fire_and_forget("chassis wheel w1 0 w2 0 w3 0 w4 0");
    tcp_client_->send_fire_and_forget("robotic_arm stop");
    if (enable_video_) {
      tcp_client_->send_fire_and_forget("stream off");
    }
    tcp_client_->disconnect();
    tcp_client_.reset();
  }
  RCLCPP_INFO(get_logger(), "deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
HardwareInterface::read(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration & /*period*/) {
  // No real feedback available on this connection (see class comment).
  // Echo commanded velocity so downstream consumers (e.g. a mecanum
  // odometry node) at least see a consistent, non-stale value.
  wheel_velocity_state_ = wheel_velocity_command_;

  if (tcp_client_) {
    tcp_client_->drain_responses();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
HardwareInterface::write(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/) {
  if (!tcp_client_ || !tcp_client_->is_connected()) {
    return hardware_interface::return_type::ERROR;
  }

  const int w1 = rad_per_sec_to_rpm(wheel_velocity_command_[kFrontRight]);
  const int w2 = rad_per_sec_to_rpm(wheel_velocity_command_[kFrontLeft]);
  const int w3 = rad_per_sec_to_rpm(wheel_velocity_command_[kRearRight]);
  const int w4 = rad_per_sec_to_rpm(wheel_velocity_command_[kRearLeft]);

  std::ostringstream cmd;
  cmd << "chassis wheel w1 " << w1 << " w2 " << w2 << " w3 " << w3 << " w4 "
      << w4;

  // Fire-and-forget: do not block the RT write() cycle on the robot's
  // ack. See TcpClient::send_fire_and_forget doc comment.
  if (!tcp_client_->send_fire_and_forget(cmd.str())) {
    RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, 1000,
                          "send_fire_and_forget failed - link down?");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

} // namespace robomaster_tether

PLUGINLIB_EXPORT_CLASS(robomaster_tether::HardwareInterface,
                       hardware_interface::SystemInterface)