#!/usr/bin/env python3
"""Operator dashboard: static UI + WebSocket teleop / state bridge.

Serves the www/ UI on :8090. Browser clients send twist / arm commands over a
WebSocket; this node publishes /cmd_vel_teleop and calls robomaster_arm actions.
Layout (sim vs tether) is chosen from $SIM via GET /api/config.

Also streams live robot state (joints, odom, arm, gripper) back to clients and
serves the processed URDF plus CAD meshes for the 3D reconstruction widget.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

import rclpy
from ament_index_python.packages import get_package_share_directory
from aiohttp import web
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robomaster_arm.action import MoveArm, SetGripper
from robomaster_arm.msg import ArmState, GripperState

# Keep in sync with robomaster_arm/scripts/arm_kinematics.py PRESETS.
PRESETS = {
    "tuck": (-0.15, 0.08),
    "extend": (0.105, 0.142),
}
JOG = 0.02
DEFAULT_SPEED = 0.3
DEFAULT_TURN = 0.8
DEADMAN_SEC = 0.4
STATE_HZ = 20.0

# Workspace envelope for the arm setpoint UI (metres in arm_base_link).
ARM_LIMITS = {
    "x_min": -0.16,
    "x_max": 0.24,
    "z_min": 0.05,
    "z_max": 0.26,
}


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("robomaster_dashboard")
        self.declare_parameter("port", 8090)
        self.declare_parameter("video_base", "http://localhost:8080")

        self._pub = self.create_publisher(Twist, "/cmd_vel_teleop", 1)
        self._move_arm = ActionClient(self, MoveArm, "/robomaster_arm/move_arm")
        self._set_gripper = ActionClient(self, SetGripper, "/robomaster_arm/set_gripper")

        self._lock = threading.Lock()
        self._arm_deadline = 0.0
        self._last_cmd = 0.0
        # Not named _clients: rclpy.Node.clients is a generator of service clients.
        self._ws_count = 0
        self._ws_clients: Set[web.WebSocketResponse] = set()
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        self.create_timer(0.05, self._deadman_tick)

        sim_raw = (os.environ.get("SIM") or "").strip().lower()
        self._sim = sim_raw == "true"
        self._port = int(self.get_parameter("port").value)
        self._video_base = str(self.get_parameter("video_base").value).rstrip("/")

        share = Path(get_package_share_directory("robomaster_dashboard"))
        self._www = share / "www"
        if not self._www.is_dir():
            # Symlink-install / source-tree fallback during development.
            self._www = Path(__file__).resolve().parent.parent / "www"

        try:
            desc_share = Path(get_package_share_directory("robomaster_description"))
            self._mesh_dir = desc_share / "meshes"
        except Exception:  # noqa: BLE001
            self._mesh_dir = Path()
            self.get_logger().warning("robomaster_description share not found; /meshes/ disabled")

        # --- State aggregator ---
        self._robot_description = ""
        self._joints: Dict[str, float] = {}
        self._authoritative_joints: Set[str] = set()
        self._odom: Optional[Dict[str, float]] = None
        self._arm: Optional[Dict[str, Any]] = None
        self._gripper_state: Optional[Dict[str, Any]] = None

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(String, "/robot_description", self._on_robot_description, latched)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(
            Odometry, "/mecanum_drive_controller/odometry", self._on_odom, 10
        )
        self.create_subscription(ArmState, "/robomaster_arm/arm_state", self._on_arm_state, 10)
        self.create_subscription(
            GripperState, "/robomaster_arm/gripper_state", self._on_gripper_state, 10
        )

        self.get_logger().info(
            f"dashboard on :{self._port} (sim={self._sim}, www={self._www})"
        )

    # ------------------------------------------------------------------ ROS callbacks

    def _on_robot_description(self, msg: String) -> None:
        with self._lock:
            self._robot_description = msg.data

    def _on_joint_states(self, msg: JointState) -> None:
        # Prefer joint_state_broadcaster (non-empty velocity) over the
        # joint_state_publisher zeros that otherwise flicker the wheels.
        authoritative = len(msg.velocity) > 0
        with self._lock:
            for i, name in enumerate(msg.name):
                if i >= len(msg.position):
                    break
                if authoritative:
                    self._joints[name] = float(msg.position[i])
                    self._authoritative_joints.add(name)
                elif name not in self._authoritative_joints:
                    self._joints[name] = float(msg.position[i])

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Yaw from quaternion (ROS ENU, z-up).
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._odom = {
                "x": float(p.x),
                "y": float(p.y),
                "z": float(p.z),
                "yaw": float(yaw),
            }

    def _on_arm_state(self, msg: ArmState) -> None:
        with self._lock:
            self._arm = {
                "x": float(msg.x),
                "z": float(msg.z),
                "arm_1": float(msg.arm_1),
                "arm_2": float(msg.arm_2),
                "moving": bool(msg.moving),
            }
            if not self._sim:
                # Tether ros2_control has no arm joints — overlay from ArmState.
                self._joints["arm_1_joint"] = float(msg.arm_1)
                self._joints["arm_2_joint"] = float(msg.arm_2)
                self._authoritative_joints.add("arm_1_joint")
                self._authoritative_joints.add("arm_2_joint")

    def _on_gripper_state(self, msg: GripperState) -> None:
        with self._lock:
            self._gripper_state = {
                "state": int(msg.state),
                "opening": float(msg.opening),
            }
            if not self._sim:
                self._joints["gripper_m_joint"] = float(msg.opening)
                self._authoritative_joints.add("gripper_m_joint")

    def state_payload(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "type": "state",
                "sim": self._sim,
                "fidelity": "ground_truth" if self._sim else "estimated",
                "joints": dict(self._joints),
                "odom": None if self._odom is None else dict(self._odom),
                "arm": None if self._arm is None else dict(self._arm),
                "gripper": (
                    None
                    if self._gripper_state is None
                    else dict(self._gripper_state)
                ),
            }

    def robot_description(self) -> str:
        with self._lock:
            return self._robot_description

    # ------------------------------------------------------------------ Teleop

    def _deadman_tick(self) -> None:
        with self._lock:
            stale = (time.monotonic() - self._last_cmd) > DEADMAN_SEC
            idle = self._ws_count == 0
        if stale or idle:
            self._pub.publish(Twist())

    def _note_cmd(self) -> None:
        with self._lock:
            self._last_cmd = time.monotonic()

    def publish_twist(self, lx: float, ly: float, az: float, speed: float, turn: float) -> None:
        tw = Twist()
        tw.linear.x = float(lx) * speed
        tw.linear.y = float(ly) * speed
        tw.angular.z = float(az) * turn
        self._pub.publish(tw)
        self._note_cmd()

    def stop(self) -> None:
        self._pub.publish(Twist())
        self._note_cmd()

    def _arm_goal(self, x: float, z: float, absolute: bool) -> None:
        now = time.monotonic()
        with self._lock:
            if now < self._arm_deadline:
                return
            self._arm_deadline = now + 10.0

        if not self._move_arm.wait_for_server(timeout_sec=0.5):
            self.get_logger().warning("move_arm action not available")
            with self._lock:
                self._arm_deadline = 0.0
            return

        goal = MoveArm.Goal()
        goal.x = float(x)
        goal.z = float(z)
        goal.absolute = bool(absolute)
        fut = self._move_arm.send_goal_async(goal)

        def _clear() -> None:
            with self._lock:
                self._arm_deadline = 0.0

        def _on_goal(f) -> None:
            try:
                gh = f.result()
            except Exception as exc:  # noqa: BLE001
                _clear()
                self.get_logger().warning(f"arm send failed: {exc}")
                return
            if gh is None or not gh.accepted:
                _clear()
                return
            gh.get_result_async().add_done_callback(lambda _rf: _clear())

        fut.add_done_callback(_on_goal)

    def _gripper(self, open_cmd: bool) -> None:
        # Discovery is driven by the asyncio spin_once loop; a short wait here
        # is only a readiness gate (same pattern as move_arm).
        if not self._set_gripper.server_is_ready():
            if not self._set_gripper.wait_for_server(timeout_sec=2.0):
                self.get_logger().warning("set_gripper action not available")
                return
        goal = SetGripper.Goal()
        goal.command = SetGripper.Goal.OPEN if open_cmd else SetGripper.Goal.CLOSE
        goal.force_level = 1
        fut = self._set_gripper.send_goal_async(goal)

        def _on_goal(f) -> None:
            try:
                gh = f.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"gripper send failed: {exc}")
                return
            if gh is None or not gh.accepted:
                self.get_logger().warning("gripper goal rejected")

        fut.add_done_callback(_on_goal)
        self.get_logger().info(
            "gripper: " + ("open" if open_cmd else "close")
        )

    def handle_arm(self, action: str) -> None:
        if action == "x+":
            self._arm_goal(JOG, 0.0, False)
        elif action == "x-":
            self._arm_goal(-JOG, 0.0, False)
        elif action == "z+":
            self._arm_goal(0.0, JOG, False)
        elif action == "z-":
            self._arm_goal(0.0, -JOG, False)
        elif action.startswith("preset_"):
            name = action.removeprefix("preset_")
            if name in PRESETS:
                x, z = PRESETS[name]
                self._arm_goal(x, z, True)
        elif action == "grip_open":
            self._gripper(True)
        elif action == "grip_close":
            self._gripper(False)

    def handle_arm_goto(self, x: float, z: float) -> None:
        self._arm_goal(x, z, True)

    def config_dict(self) -> dict:
        if self._sim:
            primary = {
                "slot": "primary",
                "topic": "/camera/overview",
                "title": "Overhead View",
            }
        else:
            primary = {
                "slot": "primary",
                "topic": "/camera/image_raw",
                "title": "Robot Camera",
            }
        return {
            "sim": self._sim,
            "video_base": self._video_base,
            "topics": {
                "raw": "/camera/image_raw",
                "overview": "/camera/overview",
                "annotated": "/camera/image_annotated",
            },
            "cameras": [
                primary,
                {
                    "slot": "annotated",
                    "topic": "/camera/image_annotated",
                    "title": "Annotated Detections",
                },
            ],
            "presets": {name: {"x": xz[0], "z": xz[1]} for name, xz in PRESETS.items()},
            "arm_limits": dict(ARM_LIMITS),
            "speed": DEFAULT_SPEED,
            "turn": DEFAULT_TURN,
        }

    # ------------------------------------------------------------------ WS helpers

    def register_ws(self, ws: web.WebSocketResponse) -> int:
        with self._lock:
            self._ws_clients.add(ws)
            self._ws_count = len(self._ws_clients)
            return self._ws_count

    def unregister_ws(self, ws: web.WebSocketResponse) -> int:
        with self._lock:
            self._ws_clients.discard(ws)
            self._ws_count = len(self._ws_clients)
            return self._ws_count

    def ws_snapshot(self) -> List[web.WebSocketResponse]:
        with self._lock:
            return list(self._ws_clients)

    def set_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop


def _build_app(node: DashboardNode) -> web.Application:
    app = web.Application()

    async def index(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(node._www / "index.html")

    async def api_config(_request: web.Request) -> web.Response:
        return web.json_response(node.config_dict())

    async def api_robot_description(_request: web.Request) -> web.Response:
        urdf = node.robot_description()
        if not urdf:
            return web.Response(
                status=503,
                text="robot_description not available yet",
                content_type="text/plain",
            )
        return web.Response(text=urdf, content_type="application/xml")

    async def ws_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        connected = node.register_ws(ws)
        node.get_logger().info(f"dashboard client connected ({connected})")
        try:
            async for msg in ws:
                if msg.type != web.WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                kind = data.get("type")
                if kind == "twist":
                    node.publish_twist(
                        data.get("lx", 0.0),
                        data.get("ly", 0.0),
                        data.get("az", 0.0),
                        float(data.get("speed", DEFAULT_SPEED)),
                        float(data.get("turn", DEFAULT_TURN)),
                    )
                elif kind == "stop":
                    node.stop()
                elif kind == "arm":
                    action = data.get("action", "")
                    if action:
                        # Action clients are sync; don't block the event loop hard.
                        await asyncio.get_event_loop().run_in_executor(
                            None, node.handle_arm, action
                        )
                elif kind == "arm_goto":
                    try:
                        x = float(data.get("x"))
                        z = float(data.get("z"))
                    except (TypeError, ValueError):
                        continue
                    await asyncio.get_event_loop().run_in_executor(
                        None, node.handle_arm_goto, x, z
                    )
        finally:
            remaining = node.unregister_ws(ws)
            node.stop()
            node.get_logger().info(f"dashboard client disconnected ({remaining})")
        return ws

    app.router.add_get("/", index)
    app.router.add_get("/api/config", api_config)
    app.router.add_get("/api/robot_description", api_robot_description)
    app.router.add_get("/ws", ws_handler)

    # Meshes first so the prefix is not shadowed by a www catch-all.
    if node._mesh_dir.is_dir():
        app.router.add_static(
            "/meshes/", node._mesh_dir, name="meshes", follow_symlinks=True
        )

    # Static assets: style.css at root, JS modules under /js/.
    async def style_css(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(node._www / "style.css")

    app.router.add_get("/style.css", style_css)
    js_dir = node._www / "js"
    if js_dir.is_dir():
        app.router.add_static("/js/", js_dir, name="js", follow_symlinks=True)

    return app


async def _broadcast_state(node: DashboardNode) -> None:
    period = 1.0 / STATE_HZ
    while rclpy.ok():
        payload = json.dumps(node.state_payload())
        for ws in node.ws_snapshot():
            if ws.closed:
                continue
            try:
                await ws.send_str(payload)
            except ConnectionResetError:
                pass
            except Exception:  # noqa: BLE001
                pass
        await asyncio.sleep(period)


def main() -> None:
    rclpy.init()
    node = DashboardNode()

    app = _build_app(node)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    node.set_loop(loop)

    async def ros_spin() -> None:
        # Cooperative spin alongside aiohttp — avoids a second executor thread
        # fighting rclpy's signal/context lifecycle.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            await asyncio.sleep(0.02)

    async def run() -> None:
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", node._port)
        await site.start()
        node.get_logger().info(f"listening on 0.0.0.0:{node._port}")
        spin_task = asyncio.create_task(ros_spin())
        state_task = asyncio.create_task(_broadcast_state(node))
        try:
            await spin_task
        except asyncio.CancelledError:
            pass
        finally:
            state_task.cancel()
            try:
                await state_task
            except asyncio.CancelledError:
                pass
            node.stop()
            await runner.cleanup()

    try:
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    main()
