#ifndef ROBOMASTER_DRIVER__SDK_BRIDGE_HPP_
#define ROBOMASTER_DRIVER__SDK_BRIDGE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robomaster_driver/srv/arm_sdk.hpp"
#include "robomaster_driver/srv/gripper_sdk.hpp"
#include "robomaster_driver/tcp_client.hpp"

namespace robomaster_driver {

// ROS service facade over a shared TcpClient for robotic_arm / robotic_gripper.
// Used by HardwareInterface (wheels own the socket) and by the standalone
// sdk_bridge node (arm-only / camera+arm without wheels).
class SdkBridge {
public:
  SdkBridge(rclcpp::Node::SharedPtr node, TcpClient *client);

private:
  void handle_arm(
      const std::shared_ptr<srv::ArmSdk::Request> request,
      std::shared_ptr<srv::ArmSdk::Response> response);
  void handle_gripper(
      const std::shared_ptr<srv::GripperSdk::Request> request,
      std::shared_ptr<srv::GripperSdk::Response> response);

  TcpClient *client_;
  rclcpp::Service<srv::ArmSdk>::SharedPtr arm_srv_;
  rclcpp::Service<srv::GripperSdk>::SharedPtr gripper_srv_;
};

} // namespace robomaster_driver

#endif // ROBOMASTER_DRIVER__SDK_BRIDGE_HPP_
