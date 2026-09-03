#!/usr/bin/env python3
"""Decodes the RoboMaster EP's H.264 stream into sensor_msgs/Image.

The tether-side counterpart to the Gazebo camera sensor: publishes the same
/camera/image_raw + /camera/camera_info pair, so object detection and the overlay
work against either backend unchanged.

Someone has to send "stream on" over the control port, which takes one client
at a time. Under `make tether` robomaster_tether owns that socket and arms the stream
itself, so this node runs with arm_stream:=false and reads video only. Standalone
(no tether backend), it arms the stream itself — see the arm_stream param.

Decoding runs on a background thread: PyAV's decode() blocks, and doing that
in a timer would stall the executor and stop the node responding to shutdown.
"""
import threading

import av
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from robomaster_camera.video_stream import StreamError, VideoStream, robot_ip


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.declare_parameter("frame_id", "camera_optical_link")
        # False when robomaster_tether already armed the stream (the tether path).
        # True to arm it here, which needs the control port to be free.
        self.declare_parameter("arm_stream", True)
        # Intrinsics for the real camera. Defaults are a plausible guess for the
        # EP's ~74 deg FOV at 1280x720, NOT a calibration: tag *detection* works
        # with them, but tag *pose* (range/bearing) will be wrong until you run
        # camera_calibration and set these. Sim doesn't need any of this —
        # Gazebo derives exact intrinsics from the sensor definition.
        self.declare_parameter("calibrated", False)
        self.declare_parameter("fx", 850.0)
        self.declare_parameter("fy", 850.0)
        self.declare_parameter("cx", 640.0)
        self.declare_parameter("cy", 360.0)
        self.declare_parameter("distortion", [0.0, 0.0, 0.0, 0.0, 0.0])

        self._frame_id = self.get_parameter("frame_id").value
        if not self.get_parameter("calibrated").value:
            self.get_logger().warn(
                "using uncalibrated default intrinsics: tag detection will work "
                "but tag pose will be wrong. Run camera_calibration and set "
                "fx/fy/cx/cy, then set calibrated:=true to silence this."
            )

        self._bridge = CvBridge()
        self._image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        self._stream = VideoStream(robot_ip(), arm=self.get_parameter("arm_stream").value)
        self._stream.open()
        self.get_logger().info("video stream open, decoding H.264 ...")

        self._running = True
        self._thread = threading.Thread(target=self._decode_loop, daemon=True)
        self._thread.start()

    def _camera_info(self, stamp, width, height) -> CameraInfo:
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = list(self.get_parameter("distortion").value)
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _decode_loop(self):
        # PyAV pulls from this generator as if it were a file; it blocks in
        # recv() until the robot sends more, which is why this isn't a timer.
        def byte_source():
            while self._running:
                chunk = self._stream.read()
                if not chunk:
                    self.get_logger().warn("video socket closed by robot.")
                    return
                yield chunk

        try:
            container = av.open(_GeneratorReader(byte_source()), format="h264", mode="r")
            for frame in container.decode(video=0):
                if not self._running:
                    break
                image = self._bridge.cv2_to_imgmsg(frame.to_ndarray(format="bgr8"), "bgr8")
                # Stamped on arrival, not from the frame's PTS: the SDK gives no
                # capture time, so this includes decode latency (tens of ms).
                # Fine for a P-controller, not for tight sensor fusion.
                stamp = self.get_clock().now().to_msg()
                image.header.stamp = stamp
                image.header.frame_id = self._frame_id
                self._image_pub.publish(image)
                self._info_pub.publish(self._camera_info(stamp, image.width, image.height))
        except Exception as exc:
            if self._running:
                self.get_logger().error(f"decode loop died: {exc}")

    def destroy_node(self):
        self._running = False
        self._stream.close()
        super().destroy_node()


class _GeneratorReader:
    """Adapts a bytes generator to the file-like read() PyAV expects."""

    def __init__(self, gen):
        self._gen = gen
        self._buf = b""

    def read(self, size):
        while len(self._buf) < size:
            try:
                self._buf += next(self._gen)
            except StopIteration:
                break
        chunk, self._buf = self._buf[:size], self._buf[size:]
        return chunk


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except StreamError as exc:
        # Connection/handshake problems are expected operator errors — report
        # the reason, not a traceback.
        rclpy.logging.get_logger("camera_node").error(str(exc))
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
