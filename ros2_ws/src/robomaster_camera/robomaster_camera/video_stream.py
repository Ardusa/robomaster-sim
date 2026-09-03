"""Socket plumbing for the RoboMaster EP's H.264 video stream.

Kept apart from camera_node.py, and free of rclpy, so the SDK handshake can be
read and debugged without the ROS layer in the way — same split as
connection_test.cpp for the control port.
"""
import os
import socket

CONTROL_PORT = 40923
VIDEO_PORT = 40921
RECV_CHUNK = 4096


class StreamError(RuntimeError):
    """Handshake or connection failure, with a message worth showing a human."""


def robot_ip() -> str:
    ip = os.environ.get("ROBOMASTER_IP")
    if not ip:
        raise StreamError(
            "ROBOMASTER_IP is not set. Set it in .env (direct-connect AP mode "
            "is usually 192.168.2.1)."
        )
    return ip


def send_and_expect(sock: socket.socket, cmd: str, expect: str = "ok") -> None:
    sock.sendall((cmd + ";").encode("utf-8"))
    resp = sock.recv(256).decode("utf-8", errors="replace").strip(";\r\n")
    if resp != expect:
        raise StreamError(f"'{cmd}' failed: expected '{expect}', got '{resp}'")


class VideoStream:
    """Opens the video socket and yields raw H.264 bytes.

    `arm` controls who turns the stream on. The control port takes one client
    at a time, so:
      arm=True  - standalone: this opens the control port and sends "stream on".
                  Fails if anything else holds it (e.g. a running `make tether`).
      arm=False - robomaster_tether's hardware interface already armed the stream on
                  the socket it owns; only the video port is touched here.
    """

    def __init__(self, ip: str, timeout: float = 5.0, arm: bool = True):
        self._ip = ip
        self._timeout = timeout
        self._arm = arm
        self._control = None
        self._video = None

    def open(self) -> None:
        if self._arm:
            self._open_control()

        self._video = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._video.settimeout(self._timeout)
        try:
            self._video.connect((self._ip, VIDEO_PORT))
        except OSError as exc:
            self._video.close()
            self._video = None
            self.close()
            hint = (
                ""
                if self._arm
                else " Nothing armed the stream: is robomaster_tether running with "
                "enable_video true?"
            )
            raise StreamError(
                f"video port {self._ip}:{VIDEO_PORT} unreachable ({exc}).{hint}"
            ) from exc

    def _open_control(self) -> None:
        self._control = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._control.settimeout(3.0)
        try:
            self._control.connect((self._ip, CONTROL_PORT))
            send_and_expect(self._control, "command")
            send_and_expect(self._control, "stream on")
        except (OSError, StreamError) as exc:
            self._control.close()
            self._control = None
            raise StreamError(
                f"cannot open control port {self._ip}:{CONTROL_PORT} ({exc}). "
                f"Either no robot there (join its Wi-Fi / check ROBOMASTER_IP), "
                f"or something else holds it - only one client is allowed, and "
                f"`make tether` keeps it for the whole session."
            ) from exc

    def read(self, size: int = RECV_CHUNK) -> bytes:
        """Returns b'' when the robot closes the socket."""
        return self._video.recv(size)

    def close(self) -> None:
        if self._control is not None:
            try:
                send_and_expect(self._control, "stream off")
            except Exception:
                pass  # best-effort, we're exiting regardless
        if self._video is not None:
            self._video.close()
            self._video = None
        if self._control is not None:
            self._control.close()
            self._control = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_exc):
        self.close()
