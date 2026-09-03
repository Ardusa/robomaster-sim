#ifndef ROBOMASTER_TETHER__TCP_CLIENT_HPP_
#define ROBOMASTER_TETHER__TCP_CLIENT_HPP_

#include <mutex>
#include <string>

namespace robomaster_tether {

// Thin wrapper around the DJI RoboMaster plaintext SDK control-command
// socket (TCP port 40923). One instance owns one control-command
// connection. Not used for video (40921) or telemetry push (40924) -
// those are separate connections with separate framing, see
// docs/plaintext_sdk_notes.md.
//
// Protocol, per robomaster-dev.readthedocs.io/en/latest/text_sdk/:
//   - every command is ASCII, terminated with ';'
//   - every command gets exactly one response, also ';'-terminated
//   - first command on a fresh connection must be "command;", robot
//     replies "ok;" and only then accepts further commands
class TcpClient {
public:
  TcpClient() = default;
  ~TcpClient();

  // Disable copy; this owns a raw socket fd.
  TcpClient(const TcpClient &) = delete;
  TcpClient &operator=(const TcpClient &) = delete;

  // Connects to <ip>:<port> (default control-command port 40923) and
  // sends "command;" to enter SDK mode. Returns false on any failure
  // (unreachable host, connect timeout, or robot doesn't reply "ok;").
  bool connect(const std::string &ip, int port = 40923, int timeout_ms = 3000);

  // Sends "quit;" (best-effort) and closes the socket.
  void disconnect();

  bool is_connected() const { return socket_fd_ >= 0; }

  // Sends `cmd` with a trailing ';' appended (don't include it yourself)
  // and blocks for the single-line response. Returns false on socket
  // error or timeout; on success `out_response` holds the reply with
  // the trailing ';' and newline stripped.
  //
  // NOTE: this blocks on recv(). Fine for setup/config commands issued
  // from on_activate(); NOT fine to call from the ros2_control write()
  // hot path at controller rate. Use send_fire_and_forget() there.
  bool send_command(const std::string &cmd, std::string &out_response,
                    int timeout_ms = 1000);

  // Sends `cmd` with a trailing ';' appended, does not wait for or
  // read the response. Used from the real-time write() loop so a slow
  // or dropped ack never stalls the control loop. Responses pile up in
  // the kernel recv buffer and are drained lazily by drain_responses().
  bool send_fire_and_forget(const std::string &cmd);

  // Non-blocking: reads and discards whatever responses are sitting in
  // the socket buffer from prior send_fire_and_forget() calls, so the
  // buffer doesn't grow unbounded over a long session. Call this once
  // per read() cycle.
  void drain_responses();

private:
  // Caller must hold send_mutex_. Discards stale fire-and-forget acks so the
  // next blocking send_command() reads its own reply, not a chassis leftover.
  void drain_responses_unlocked();

  int socket_fd_ = -1;
  std::mutex
      send_mutex_; // serializes writers; recv is single-threaded by design
};

} // namespace robomaster_tether

#endif // ROBOMASTER_TETHER__TCP_CLIENT_HPP_