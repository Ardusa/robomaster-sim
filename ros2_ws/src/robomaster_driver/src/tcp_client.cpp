#include "robomaster_driver/tcp_client.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace robomaster_driver {

namespace {
auto get_logger() { return rclcpp::get_logger("tcp_client"); }

// Sets SO_RCVTIMEO / SO_SNDTIMEO so blocking recv()/send() calls give
// up after timeout_ms instead of hanging forever on a dead link.
void set_socket_timeout(int fd, int timeout_ms) {
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}
} // namespace

TcpClient::~TcpClient() { disconnect(); }

bool TcpClient::connect(const std::string &ip, int port,
                                  int timeout_ms) {
  if (socket_fd_ >= 0) {
    RCLCPP_WARN(
        get_logger(),
        "connect() called while already connected; disconnecting first.");
    disconnect();
  }

  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "socket() failed: %s", std::strerror(errno));
    return false;
  }

  set_socket_timeout(socket_fd_, timeout_ms);

  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    RCLCPP_ERROR(get_logger(), "invalid robot IP address: %s", ip.c_str());
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(get_logger(),
              "connecting to RoboMaster control port at %s:%d ...", ip.c_str(),
              port);
  if (::connect(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr),
                sizeof(addr)) != 0) {
    RCLCPP_ERROR(get_logger(), "TCP connect() failed: %s",
                 std::strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Enter SDK mode. Robot will not accept any other command until this
  // succeeds. See protocol_api.html#enter-sdk-mode.
  std::string response;
  if (!send_command("command", response, timeout_ms) || response != "ok") {
    RCLCPP_ERROR(get_logger(),
                 "robot did not enter SDK mode (got '%s', expected 'ok')",
                 response.c_str());
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(get_logger(), "connected, robot is in SDK mode.");
  return true;
}

void TcpClient::disconnect() {
  if (socket_fd_ < 0) {
    return;
  }
  // Fire-and-forget, not send_command: we're closing the socket regardless, so
  // waiting on a reply only buys a guaranteed timeout and a red ERROR line that
  // makes a clean shutdown look like a failure. The robot drops SDK mode when
  // the connection closes anyway; "quit" is just the polite version.
  send_fire_and_forget("quit");
  close(socket_fd_);
  socket_fd_ = -1;
}

bool TcpClient::send_command(const std::string &cmd,
                                       std::string &out_response,
                                       int timeout_ms) {
  if (socket_fd_ < 0) {
    return false;
  }
  std::lock_guard<std::mutex> lock(send_mutex_);

  set_socket_timeout(socket_fd_, timeout_ms);

  const std::string wire_cmd = cmd + ";";
  if (send(socket_fd_, wire_cmd.c_str(), wire_cmd.size(), 0) < 0) {
    RCLCPP_ERROR(get_logger(), "send() failed for '%s': %s", cmd.c_str(),
                 std::strerror(errno));
    return false;
  }

  char buf[256];
  const ssize_t n = recv(socket_fd_, buf, sizeof(buf) - 1, 0);
  if (n <= 0) {
    RCLCPP_ERROR(get_logger(),
                 "recv() failed/timed out waiting for reply to '%s': %s",
                 cmd.c_str(), std::strerror(errno));
    return false;
  }
  buf[n] = '\0';

  out_response.assign(buf);
  // Strip trailing ';' and any CR/LF the robot appends.
  while (!out_response.empty() &&
         (out_response.back() == ';' || out_response.back() == '\n' ||
          out_response.back() == '\r')) {
    out_response.pop_back();
  }
  return true;
}

bool TcpClient::send_fire_and_forget(const std::string &cmd) {
  if (socket_fd_ < 0) {
    return false;
  }
  std::lock_guard<std::mutex> lock(send_mutex_);
  const std::string wire_cmd = cmd + ";";
  if (send(socket_fd_, wire_cmd.c_str(), wire_cmd.size(), 0) < 0) {
    RCLCPP_ERROR(get_logger(), "send_fire_and_forget() failed for '%s': %s",
                 cmd.c_str(), std::strerror(errno));
    return false;
  }
  return true;
}

void TcpClient::drain_responses() {
  if (socket_fd_ < 0) {
    return;
  }
  char buf[1024];
  // MSG_DONTWAIT: never block the read() cycle, just skim whatever is
  // already sitting in the kernel buffer.
  while (true) {
    const ssize_t n = recv(socket_fd_, buf, sizeof(buf), MSG_DONTWAIT);
    if (n <= 0) {
      break;
    }
  }
}

} // namespace robomaster_driver