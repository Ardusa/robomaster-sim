// Minimal standalone connectivity check. Not a ROS2 node - deliberately
// has no rclcpp::Node, no controller_manager, no URDF dependency. Run
// this FIRST before wiring up the full ros2_control stack: it tells
// you in about two seconds whether the robot is reachable at all,
// which collapses a lot of otherwise-ambiguous failure modes (wrong
// Wi-Fi network, robot not in direct-connection mode, robot not
// powered on, wrong IP) down to one clear pass/fail.
//
// Usage:
//   ros2 run robomaster_driver connection_test [robot_ip]
//   (defaults to the IP set in .env)
//
// What it does, in order:
//   1. TCP connect to <ip>:40923
//   2. send "command;", expect "ok;"          -> confirms SDK mode
//   3. query "robot battery ?;"                -> confirms two-way traffic
//   4. send a zero-velocity chassis command     -> confirms write path,
//      without actually moving the robot
//   5. cleanly send "quit;" and disconnect
#include <iostream>
#include <string>

#include "robomaster_driver/tcp_client.hpp"

int main(int  /*argc*/, char ** /*argv*/) {
  // Not std::string x = std::getenv(...): that's UB (a segfault, not an error)
  // when the var is unset, which is exactly when you'd be running this.
  const char *ip_env = std::getenv("ROBOMASTER_IP");
  if (ip_env == nullptr || *ip_env == '\0') {
    std::cerr << "ROBOMASTER_IP is not set. Set it in .env (direct-connect AP "
                 "mode is usually 192.168.2.1).\n";
    return 2;
  }
  const std::string robot_ip = ip_env;

  std::cout << "RoboMaster EP connection test\n";
  std::cout << "  target: " << robot_ip << ":40923\n";
  std::cout
      << "  make sure you're joined to the robot's Wi-Fi hotspot and it's\n";
  std::cout << "  in direct-connection mode (switch on the smart central "
               "control)\n\n";

  robomaster_driver::TcpClient client;

  std::cout << "[1/4] connecting + entering SDK mode... ";
  if (!client.connect(robot_ip)) {
    std::cout << "FAILED\n";
    std::cout << "\nCouldn't connect. Check: robot powered on, "
                 "direct-connection mode,\n";
    std::cout << "PC joined to the robot's Wi-Fi hotspot, IP is " << robot_ip << ".\n";
    return 1;
  }
  std::cout << "ok\n";

  // Retried, and with a longer timeout than the 1s default: this is the first
  // command that has to round-trip past the SDK layer to the chassis, and it
  // intermittently overruns 1s - especially right after a reconnect, while the
  // robot is still tearing down a previous session (it allows one client, so a
  // container restart can leave it briefly busy). A false FAILED here sends you
  // hunting a network problem that isn't there.
  std::cout << "[2/4] querying battery level... ";
  std::string response;
  bool battery_ok = client.send_command("robot battery ?", response, 3000);
  if (!battery_ok) {
    std::cout << "no response, retrying... ";
    battery_ok = client.send_command("robot battery ?", response, 3000);
  }
  if (!battery_ok) {
    std::cout << "FAILED (no response)\n";
    std::cout << "\nSDK mode was entered but the robot never answered. If you "
                 "just restarted\nthe container, wait a few seconds for the old "
                 "session to drop and retry.\n";
    return 1;
  }
  std::cout << response << "%\n";

  std::cout << "[3/4] setting movement mode to free... ";
  if (!client.send_command("robot mode free", response) || response != "ok") {
    std::cout << "FAILED (got '" << response << "')\n";
    return 1;
  }
  std::cout << "ok\n";

  std::cout << "[4/4] sending zero-velocity chassis command (write-path check, "
               "robot should NOT move)... ";
  if (!client.send_command("chassis wheel w1 0 w2 0 w3 0 w4 0", response) ||
      response != "ok") {
    std::cout << "FAILED (got '" << response << "')\n";
    return 1;
  }
  std::cout << "ok\n";

  client.disconnect();
  std::cout << "\nAll checks passed. Robot is reachable and accepting SDK "
               "commands.\n";
  return 0;
}