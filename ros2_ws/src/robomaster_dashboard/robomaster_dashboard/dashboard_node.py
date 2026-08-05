#!/usr/bin/env python3
"""Operator dashboard: static UI + WebSocket teleop bridge.

Serves the www/ UI on :8090. Browser clients send twist / arm commands over a
WebSocket; this node publishes /cmd_vel_teleop and calls robomaster_arm actions.
Layout (sim vs tether) is chosen from $SIM via GET /api/config.
"""

from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from aiohttp import web
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node

from robomaster_arm.action import MoveArm, SetGripper

# Match teleop_node.py so the dashboard feels the same as the keyboard fallback.
PRESETS = {
    "tuck": (0.059, 0.190),
    "reach": (0.159, 0.171),
    "raise": (0.023, 0.229),
}
JOG = 0.02
DEFAULT_SPEED = 0.3
DEFAULT_TURN = 0.8
DEADMAN_SEC = 0.4


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

        self.get_logger().info(
            f"dashboard on :{self._port} (sim={self._sim}, www={self._www})"
        )

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
        if not self._set_gripper.wait_for_server(timeout_sec=0.5):
            self.get_logger().warning("set_gripper action not available")
            return
        goal = SetGripper.Goal()
        goal.command = SetGripper.Goal.OPEN if open_cmd else SetGripper.Goal.CLOSE
        goal.force_level = 1
        self._set_gripper.send_goal_async(goal)

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

    def config_dict(self) -> dict:
        return {
            "sim": self._sim,
            "video_base": self._video_base,
            "topics": {
                "raw": "/camera/image_raw",
                "overview": "/camera/overview",
                "annotated": "/camera/image_annotated",
            },
            "speed": DEFAULT_SPEED,
            "turn": DEFAULT_TURN,
        }


def _build_app(node: DashboardNode) -> web.Application:
    app = web.Application()

    async def index(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(node._www / "index.html")

    async def api_config(_request: web.Request) -> web.Response:
        return web.json_response(node.config_dict())

    async def ws_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        with node._lock:
            node._ws_count += 1
            connected = node._ws_count
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
        finally:
            with node._lock:
                node._ws_count = max(0, node._ws_count - 1)
                remaining = node._ws_count
            node.stop()
            node.get_logger().info(f"dashboard client disconnected ({remaining})")
        return ws

    async def app_js(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(node._www / "app.js")

    async def style_css(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(node._www / "style.css")

    app.router.add_get("/", index)
    app.router.add_get("/api/config", api_config)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/app.js", app_js)
    app.router.add_get("/style.css", style_css)
    return app


def main() -> None:
    rclpy.init()
    node = DashboardNode()

    app = _build_app(node)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

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
        try:
            await spin_task
        except asyncio.CancelledError:
            pass
        finally:
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
