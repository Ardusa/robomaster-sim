#include "robomaster_tether/sdk_bridge.hpp"

#include <sstream>

namespace robomaster_tether {

SdkBridge::SdkBridge(rclcpp::Node::SharedPtr node, TcpClient *client)
    : client_(client) {
  arm_srv_ = node->create_service<srv::ArmSdk>(
      "robomaster_tether/arm_sdk",
      [this](const std::shared_ptr<srv::ArmSdk::Request> req,
             std::shared_ptr<srv::ArmSdk::Response> res) { handle_arm(req, res); });
  gripper_srv_ = node->create_service<srv::GripperSdk>(
      "robomaster_tether/gripper_sdk",
      [this](const std::shared_ptr<srv::GripperSdk::Request> req,
             std::shared_ptr<srv::GripperSdk::Response> res) {
        handle_gripper(req, res);
      });
  RCLCPP_INFO(node->get_logger(),
              "SDK bridge services up: arm_sdk, gripper_sdk");
}

void SdkBridge::handle_arm(
    const std::shared_ptr<srv::ArmSdk::Request> request,
    std::shared_ptr<srv::ArmSdk::Response> response) {
  response->success = false;
  response->x_cm = request->x_cm;
  response->y_cm = request->y_cm;
  if (!client_ || !client_->is_connected()) {
    response->message = "not connected";
    return;
  }

  std::ostringstream cmd;
  const std::string &c = request->command;
  if (c == "moveto") {
    cmd << "robotic_arm moveto x " << request->x_cm << " y " << request->y_cm;
  } else if (c == "move") {
    cmd << "robotic_arm move x " << request->x_cm << " y " << request->y_cm;
  } else if (c == "stop") {
    cmd << "robotic_arm stop";
  } else if (c == "position") {
    cmd << "robotic_arm position ?";
  } else {
    response->message = "unknown command (use moveto|move|stop|position)";
    return;
  }

  std::string reply;
  if (!client_->send_command(cmd.str(), reply, 5000)) {
    response->message = "send failed";
    return;
  }

  if (c == "position") {
    // Reply is typically "x y" in cm.
    std::istringstream iss(reply);
    double x = 0.0, y = 0.0;
    if (iss >> x >> y) {
      response->x_cm = x;
      response->y_cm = y;
      response->success = true;
      response->message = reply;
      return;
    }
    response->message = "bad position reply: " + reply;
    return;
  }

  response->success = (reply == "ok");
  response->message = reply;
}

void SdkBridge::handle_gripper(
    const std::shared_ptr<srv::GripperSdk::Request> request,
    std::shared_ptr<srv::GripperSdk::Response> response) {
  response->success = false;
  response->state = 255;
  if (!client_ || !client_->is_connected()) {
    response->message = "not connected";
    return;
  }

  std::ostringstream cmd;
  const std::string &c = request->command;
  if (c == "open" || c == "close") {
    cmd << "robotic_gripper " << c;
    if (request->force_level >= 1 && request->force_level <= 4) {
      cmd << " " << static_cast<int>(request->force_level);
    }
  } else if (c == "status") {
    cmd << "robotic_gripper status ?";
  } else {
    response->message = "unknown command (use open|close|status)";
    return;
  }

  std::string reply;
  if (!client_->send_command(cmd.str(), reply, 5000)) {
    response->message = "send failed";
    return;
  }

  if (c == "status") {
    try {
      response->state = static_cast<uint8_t>(std::stoi(reply));
      response->success = true;
      response->message = reply;
    } catch (...) {
      response->message = "bad status reply: " + reply;
    }
    return;
  }

  response->success = (reply == "ok");
  response->message = reply;
}

} // namespace robomaster_tether
