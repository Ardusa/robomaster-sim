import { createJoystick, applyDeadzone } from "./joystick.js";
import { createArmPanel } from "./arm.js";
import { createTelemetry } from "./telemetry.js";
import { createRobot3d } from "./robot3d.js";
import { createSnapshotFeed } from "./feeds.js";

(() => {
  const statusEl = document.getElementById("status");
  const modeEl = document.getElementById("mode");
  const primaryTitle = document.getElementById("primary-title");
  const annotatedTitle = document.getElementById("annotated-title");
  const primaryFeed = createSnapshotFeed(document.getElementById("primary-feed"), {
    hz: 12,
  });
  const annotatedFeed = createSnapshotFeed(document.getElementById("annotated-feed"), {
    hz: 12,
  });
  const speedEl = document.getElementById("speed");
  const turnEl = document.getElementById("turn");
  const gamepadLabel = document.getElementById("gamepad-label");

  const driveJoy = createJoystick(document.getElementById("drive-stick"), {
    interactive: true,
  });
  const armJoy = createJoystick(document.getElementById("arm-stick"), {
    interactive: true,
  });

  let ws = null;
  let speed = 0.3;
  let turn = 0.8;
  let gamepadIndex = null;
  let lastArmMs = 0;
  const SEND_HZ = 20;
  const ARM_JOG_MS = 180;

  function setStatus(text, ok) {
    statusEl.textContent = text;
    statusEl.classList.toggle("ok", ok === true);
    statusEl.classList.toggle("bad", ok === false);
  }

  function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(obj));
    }
  }

  function sendTwist(lx, ly, az) {
    send({ type: "twist", lx, ly, az, speed, turn });
  }

  function sendArm(action) {
    const now = performance.now();
    if (now - lastArmMs < ARM_JOG_MS) return;
    lastArmMs = now;
    send({ type: "arm", action });
  }

  /** Canvas stick → arm workspace: right = +x (reach), up = +z (raise). */
  function jogArmFromStick(sx, sy) {
    const ax = applyDeadzone(sx);
    const az = applyDeadzone(-sy);
    if (ax === 0 && az === 0) return;
    if (Math.abs(ax) >= Math.abs(az)) {
      sendArm(ax > 0 ? "x+" : "x-");
    } else {
      sendArm(az > 0 ? "z+" : "z-");
    }
  }

  /** LT/RT (and D-pad L/R) for yaw — right stick is reserved for the arm. */
  function yawFromPad(pad) {
    const lt = pad.buttons[6]?.value ?? (pad.buttons[6]?.pressed ? 1 : 0);
    const rt = pad.buttons[7]?.value ?? (pad.buttons[7]?.pressed ? 1 : 0);
    let az = 0;
    if (lt > 0.1) az += lt;
    if (rt > 0.1) az -= rt;
    if (pad.buttons[14]?.pressed) az = 1; // D-pad left → +yaw (CCW)
    if (pad.buttons[15]?.pressed) az = -1; // D-pad right
    return applyDeadzone(az);
  }

  const stateListeners = [];

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
      armJoy.setLocked(false);
      gamepadLabel.textContent =
        "No gamepad — drag sticks · A=tuck · B=extend · LT/RT=turn when connected.";
      const lx = applyDeadzone(-driveJoy.stick.y);
      const ly = applyDeadzone(-driveJoy.stick.x);
      if (lx === 0 && ly === 0) {
        send({ type: "stop" });
      } else {
        sendTwist(lx, ly, 0);
      }
      jogArmFromStick(armJoy.stick.x, armJoy.stick.y);
      return;
    }

    driveJoy.setLocked(true);
    armJoy.setLocked(true);
    gamepadLabel.textContent =
      `Gamepad: ${pad.id} · left=drive · right=arm · LT/RT=turn · A=tuck · B=extend · LB/RB=gripper`;

    const lx = applyDeadzone(-(pad.axes[1] || 0));
    const ly = applyDeadzone(-(pad.axes[0] || 0));
    const az = yawFromPad(pad);
    // Right stick: axes[2]=X, axes[3]=Y (down-positive).
    const rax = applyDeadzone(pad.axes[2] || 0);
    const ray = applyDeadzone(pad.axes[3] || 0);
    driveJoy.setValue(-ly, -lx);
    armJoy.setValue(rax, ray);

    if (lx === 0 && ly === 0 && az === 0) {
      send({ type: "stop" });
    } else {
      sendTwist(lx, ly, az);
    }

    jogArmFromStick(rax, ray);

    // Face buttons: A=tuck, B=extend.
    if (pad.buttons[0]?.pressed) sendArm("preset_tuck");
    if (pad.buttons[1]?.pressed) sendArm("preset_extend");
    if (pad.buttons[4]?.pressed) sendArm("grip_open");
    if (pad.buttons[5]?.pressed) sendArm("grip_close");
    // D-pad up/down = fine arm jog (left/right already used for yaw).
    if (pad.buttons[12]?.pressed) sendArm("z+");
    if (pad.buttons[13]?.pressed) sendArm("z-");
  }

  window.addEventListener("gamepadconnected", (e) => {
    gamepadIndex = e.gamepad.index;
    gamepadLabel.textContent = `Gamepad: ${e.gamepad.id}`;
  });
  window.addEventListener("gamepaddisconnected", () => {
    gamepadIndex = null;
    driveJoy.setLocked(false);
    armJoy.setLocked(false);
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
        primaryFeed.start(cfg.video_base, cam.topic, cam.title || "Primary camera");
      } else if (cam.slot === "annotated") {
        annotatedTitle.textContent = cam.title || "Annotated Detections";
        annotatedFeed.start(
          cfg.video_base,
          cam.topic,
          cam.title || "Annotated detections"
        );
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
