// Standalone owner of the EP control port when wheels are not running.
// Offers the same arm/gripper services HardwareInterface exposes when it holds
// the socket. Also arms the video stream when enable_video is true so the
// camera can read port 40921 without opening a second control client.
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robomaster_tether/sdk_bridge.hpp"
#include "robomaster_tether/tcp_client.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("robomaster_sdk_bridge");

  node->declare_parameter<std::string>("robot_ip", "");
  node->declare_parameter<int>("control_port", 40923);
  node->declare_parameter<bool>("enable_video", true);

  std::string robot_ip = node->get_parameter("robot_ip").as_string();
  if (robot_ip.empty()) {
    const char *env = std::getenv("ROBOMASTER_IP");
    if (env != nullptr) {
      robot_ip = env;
    }
  }
  if (robot_ip.empty()) {
    RCLCPP_FATAL(node->get_logger(),
                 "robot_ip unset and ROBOMASTER_IP empty — cannot connect.");
    return 1;
  }

  const int port = node->get_parameter("control_port").as_int();
  const bool enable_video = node->get_parameter("enable_video").as_bool();

  auto client = std::make_unique<robomaster_tether::TcpClient>();
  if (!client->connect(robot_ip, port)) {
    RCLCPP_FATAL(node->get_logger(), "failed to connect at %s:%d",
                 robot_ip.c_str(), port);
    return 1;
  }

  std::string response;
  client->send_command("robot mode free", response);
  if (enable_video) {
    if (!client->send_command("stream on", response) || response != "ok") {
      RCLCPP_WARN(node->get_logger(),
                  "'stream on' failed (got '%s') — camera may have no video.",
                  response.c_str());
    }
  }

  robomaster_tether::SdkBridge bridge(node, client.get());
  RCLCPP_INFO(node->get_logger(), "sdk_bridge owning control port at %s:%d",
              robot_ip.c_str(), port);

  rclcpp::spin(node);

  if (enable_video) {
    client->send_fire_and_forget("stream off");
  }
  client->disconnect();
  rclcpp::shutdown();
  return 0;
}
