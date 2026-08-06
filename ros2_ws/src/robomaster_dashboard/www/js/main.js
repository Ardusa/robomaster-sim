import { createJoystick, applyDeadzone } from "./joystick.js";
import { createArmPanel } from "./arm.js";
import { createTelemetry } from "./telemetry.js";
import { createRobot3d } from "./robot3d.js";

(() => {
  const statusEl = document.getElementById("status");
  const modeEl = document.getElementById("mode");
  const primaryTitle = document.getElementById("primary-title");
  const annotatedTitle = document.getElementById("annotated-title");
  const primaryFeed = document.getElementById("primary-feed");
  const annotatedFeed = document.getElementById("annotated-feed");
  const speedEl = document.getElementById("speed");
  const turnEl = document.getElementById("turn");
  const gamepadLabel = document.getElementById("gamepad-label");

  const driveJoy = createJoystick(document.getElementById("drive-stick"), {
    interactive: true,
  });
  // Read-only preview for the separate arm-teleop agent's right stick.
  const armJoy = createJoystick(document.getElementById("arm-stick"), {
    interactive: false,
  });

  let ws = null;
  let speed = 0.3;
  let turn = 0.8;
  let gamepadIndex = null;
  let lastArmMs = 0;
  const SEND_HZ = 20;

  function sendArm(action) {
    const now = performance.now();
    if (now - lastArmMs < 200) return;
    lastArmMs = now;
    send({ type: "arm", action });
  }

  const stateListeners = [];

  function setStatus(text, ok) {
    statusEl.textContent = text;
    statusEl.classList.toggle("ok", ok === true);
    statusEl.classList.toggle("bad", ok === false);
  }

  function streamUrl(base, topic) {
    // web_video_server validates the raw query value; do not percent-encode '/'.
    return `${base}/stream?topic=${topic}`;
  }

  function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(obj));
    }
  }

  function sendTwist(lx, ly, az) {
    send({ type: "twist", lx, ly, az, speed, turn });
  }

  const armPanel = createArmPanel({
    presetGrid: document.getElementById("preset-grid"),
    gotoForm: document.getElementById("arm-goto"),
    gotoX: document.getElementById("goto-x"),
    gotoZ: document.getElementById("goto-z"),
    send,
  });

  const telemetry = createTelemetry({
    armX: document.getElementById("arm-x"),
    armZ: document.getElementById("arm-z"),
    armMoving: document.getElementById("arm-moving"),
    gripperLabel: document.getElementById("gripper-label"),
    fidelityBadge: document.getElementById("fidelity-badge"),
    gotoX: document.getElementById("goto-x"),
    gotoZ: document.getElementById("goto-z"),
  });
  stateListeners.push((s) => telemetry.onState(s));

  const robot3d = createRobot3d(document.getElementById("robot3d"), {
    loadingEl: document.getElementById("robot3d-loading"),
    resetBtn: document.getElementById("reset-view"),
  });
  stateListeners.push((s) => robot3d.onState(s));

  speedEl.addEventListener("input", () => {
    speed = Number(speedEl.value);
  });
  turnEl.addEventListener("input", () => {
    turn = Number(turnEl.value);
  });

  function pollGamepad() {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    let pad = null;
    if (gamepadIndex !== null && pads[gamepadIndex]) {
      pad = pads[gamepadIndex];
    } else {
      for (const p of pads) {
        if (p) {
          pad = p;
          gamepadIndex = p.index;
          break;
        }
      }
    }

    if (!pad) {
      gamepadIndex = null;
      driveJoy.setLocked(false);
      gamepadLabel.textContent =
        "No gamepad — use the mecanum stick or plug in a controller.";
      // Mouse stick: canvas y is down-positive; forward is -y.
      const lx = applyDeadzone(-driveJoy.stick.y);
      const ly = applyDeadzone(-driveJoy.stick.x);
      armJoy.setValue(0, 0);
      if (lx === 0 && ly === 0) {
        send({ type: "stop" });
      } else {
        sendTwist(lx, ly, 0);
      }
      return;
    }

    driveJoy.setLocked(true);
    gamepadLabel.textContent = `Gamepad: ${pad.id}`;
    const lx = applyDeadzone(-(pad.axes[1] || 0));
    const ly = applyDeadzone(-(pad.axes[0] || 0));
    const az = applyDeadzone(-(pad.axes[2] || 0));
    // Right stick visualizer (axes 2/3). Arm command path owned by other agent.
    const rax = applyDeadzone(pad.axes[2] || 0);
    const ray = applyDeadzone(pad.axes[3] || 0);
    driveJoy.setValue(-ly, -lx);
    armJoy.setValue(rax, ray);

    if (lx === 0 && ly === 0 && az === 0) {
      send({ type: "stop" });
    } else {
      sendTwist(lx, ly, az);
    }

    // Face buttons: A=0 B=1 X=2 Y=3 on standard mapping.
    if (pad.buttons[0]?.pressed) sendArm("preset_tuck");
    if (pad.buttons[1]?.pressed) sendArm("preset_reach");
    if (pad.buttons[2]?.pressed) sendArm("preset_raise");
    if (pad.buttons[4]?.pressed) sendArm("grip_open");
    if (pad.buttons[5]?.pressed) sendArm("grip_close");
  }

  window.addEventListener("gamepadconnected", (e) => {
    gamepadIndex = e.gamepad.index;
    gamepadLabel.textContent = `Gamepad: ${e.gamepad.id}`;
  });
  window.addEventListener("gamepaddisconnected", () => {
    gamepadIndex = null;
    driveJoy.setLocked(false);
    driveJoy.reset();
    armJoy.reset();
  });

  document.addEventListener("visibilitychange", () => {
    if (document.hidden) send({ type: "stop" });
  });

  function connectWs() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.onopen = () => setStatus("connected", true);
    ws.onclose = () => {
      setStatus("disconnected — retrying…", false);
      setTimeout(connectWs, 1000);
    };
    ws.onerror = () => setStatus("socket error", false);
    ws.onmessage = (ev) => {
      let data;
      try {
        data = JSON.parse(ev.data);
      } catch (_) {
        return;
      }
      if (data?.type === "state") {
        for (const fn of stateListeners) fn(data);
      }
    };
  }

  function applyCameras(cfg) {
    const cams = cfg.cameras || [];
    for (const cam of cams) {
      if (cam.slot === "primary") {
        primaryTitle.textContent = cam.title || "Camera";
        primaryFeed.src = streamUrl(cfg.video_base, cam.topic);
        primaryFeed.alt = cam.title || "Primary camera";
      } else if (cam.slot === "annotated") {
        annotatedTitle.textContent = cam.title || "Annotated Detections";
        annotatedFeed.src = streamUrl(cfg.video_base, cam.topic);
        annotatedFeed.alt = cam.title || "Annotated detections";
      }
    }
  }

  async function boot() {
    const cfg = await fetch("/api/config").then((r) => r.json());
    speed = cfg.speed ?? speed;
    turn = cfg.turn ?? turn;
    speedEl.value = String(speed);
    turnEl.value = String(turn);

    modeEl.textContent = cfg.sim ? "SIM" : "TETHER";
    applyCameras(cfg);
    armPanel.buildPresets(cfg.presets);
    armPanel.applyLimits(cfg.arm_limits);

    connectWs();
    setInterval(pollGamepad, 1000 / SEND_HZ);
    robot3d.load().catch((err) => {
      console.error(err);
      const el = document.getElementById("robot3d-loading");
      if (el) {
        el.hidden = false;
        el.textContent = `3D load failed: ${err.message || err}`;
      }
    });
  }

  boot().catch((err) => {
    setStatus(`failed: ${err}`, false);
  });
})();
