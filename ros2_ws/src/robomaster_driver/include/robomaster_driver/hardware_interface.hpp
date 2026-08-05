#ifndef ROBOMASTER_DRIVER__HARDWARE_INTERFACE_HPP_
#define ROBOMASTER_DRIVER__HARDWARE_INTERFACE_HPP_

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robomaster_driver/sdk_bridge.hpp"
#include "robomaster_driver/tcp_client.hpp"

namespace robomaster_driver {

// ros2_control SystemInterface for the physical DJI RoboMaster EP,
// talking to the plaintext SDK over TCP. Mirrors gz_ros2_control's
// role on the sim side: sim.launch.py loads the Gazebo plugin,
// tether.launch.py loads this one, against the same controller_manager
// config and the same four wheel joint names.
//
// Owns the single control-port client for the session and exposes
// robotic_arm / robotic_gripper services so robomaster_arm shares that
// socket instead of opening a second connection.
class HardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HardwareInterface)

  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void start_sdk_bridge();
  void stop_sdk_bridge();

  static constexpr size_t kNumWheels = 4;
  enum WheelIndex {
    kFrontRight = 0,
    kFrontLeft = 1,
    kRearRight = 2,
    kRearLeft = 3
  };

  std::array<double, kNumWheels> wheel_velocity_command_{};
  std::array<double, kNumWheels> wheel_velocity_state_{};

  double wheel_radius_m_ = 0.05;
  std::string robot_ip_;
  int control_port_ = 40923;
  bool enable_video_ = true;

  std::unique_ptr<TcpClient> tcp_client_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Arm/gripper services on the same socket as chassis writes.
  rclcpp::Node::SharedPtr sdk_node_;
  std::unique_ptr<SdkBridge> sdk_bridge_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> sdk_executor_;
  std::thread sdk_spin_thread_;
};

} // namespace robomaster_driver

#endif // ROBOMASTER_DRIVER__HARDWARE_INTERFACE_HPP_
