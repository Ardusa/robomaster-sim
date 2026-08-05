(() => {
  const statusEl = document.getElementById("status");
  const modeEl = document.getElementById("mode");
  const layoutEl = document.getElementById("layout");
  const overviewPane = document.getElementById("overview-pane");
  const overviewImg = document.getElementById("overview");
  const rawImg = document.getElementById("raw");
  const canvas = document.getElementById("joystick");
  const ctx = canvas.getContext("2d");
  const speedEl = document.getElementById("speed");
  const turnEl = document.getElementById("turn");
  const gamepadLabel = document.getElementById("gamepad-label");

  let ws = null;
  let speed = 0.3;
  let turn = 0.8;
  let stick = { x: 0, y: 0 }; // x = strafe (ly), y = forward (lx), canvas space
  let dragging = false;
  let gamepadIndex = null;
  let lastArmMs = 0;

  const DEADZONE = 0.12;
  const SEND_HZ = 20;

  function setStatus(text, ok) {
    statusEl.textContent = text;
    statusEl.classList.toggle("ok", ok === true);
    statusEl.classList.toggle("bad", ok === false);
  }

  function streamUrl(base, topic) {
    return `${base}/stream?topic=${encodeURIComponent(topic)}`;
  }

  function applyDeadzone(v) {
    return Math.abs(v) < DEADZONE ? 0 : v;
  }

  function drawJoystick() {
    const w = canvas.width;
    const h = canvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const r = Math.min(w, h) * 0.42;
    ctx.clearRect(0, 0, w, h);
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fillStyle = "#151b24";
    ctx.fill();
    ctx.strokeStyle = "#2c3644";
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(cx - r, cy);
    ctx.lineTo(cx + r, cy);
    ctx.moveTo(cx, cy - r);
    ctx.lineTo(cx, cy + r);
    ctx.strokeStyle = "#243040";
    ctx.stroke();

    const knx = cx + stick.x * r;
    const kny = cy + stick.y * r;
    ctx.beginPath();
    ctx.arc(knx, kny, r * 0.28, 0, Math.PI * 2);
    ctx.fillStyle = "#3d9cf0";
    ctx.fill();
  }

  function setStickFromPointer(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const r = Math.min(rect.width, rect.height) * 0.42;
    let dx = (clientX - cx) / r;
    let dy = (clientY - cy) / r;
    const mag = Math.hypot(dx, dy);
    if (mag > 1) {
      dx /= mag;
      dy /= mag;
    }
    stick.x = dx;
    stick.y = dy;
    drawJoystick();
  }

  function resetStick() {
    stick.x = 0;
    stick.y = 0;
    drawJoystick();
  }

  canvas.addEventListener("pointerdown", (e) => {
    if (gamepadIndex !== null) return;
    dragging = true;
    canvas.setPointerCapture(e.pointerId);
    setStickFromPointer(e.clientX, e.clientY);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!dragging || gamepadIndex !== null) return;
    setStickFromPointer(e.clientX, e.clientY);
  });
  function endDrag(e) {
    if (!dragging) return;
    dragging = false;
    try { canvas.releasePointerCapture(e.pointerId); } catch (_) { /* ignore */ }
    if (gamepadIndex === null) resetStick();
  }
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);

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
    if (now - lastArmMs < 200) return;
    lastArmMs = now;
    send({ type: "arm", action });
  }

  document.querySelectorAll("[data-arm]").forEach((btn) => {
    btn.addEventListener("click", () => sendArm(btn.dataset.arm));
  });

  speedEl.value = String(speed);
  turnEl.value = String(turn);
  speedEl.addEventListener("input", () => { speed = Number(speedEl.value); });
  turnEl.addEventListener("input", () => { turn = Number(turnEl.value); });

  function pollGamepad() {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    let pad = null;
    if (gamepadIndex !== null && pads[gamepadIndex]) {
      pad = pads[gamepadIndex];
    } else {
      for (const p of pads) {
        if (p) { pad = p; gamepadIndex = p.index; break; }
      }
    }

    if (!pad) {
      gamepadIndex = null;
      gamepadLabel.textContent =
        "No gamepad — use the joystick or plug in an Xbox controller.";
      // Mouse stick: canvas y is down-positive; forward is -y.
      const lx = applyDeadzone(-stick.y);
      const ly = applyDeadzone(-stick.x);
      if (lx === 0 && ly === 0) {
        send({ type: "stop" });
      } else {
        sendTwist(lx, ly, 0);
      }
      return;
    }

    gamepadLabel.textContent = `Gamepad: ${pad.id}`;
    const lx = applyDeadzone(-(pad.axes[1] || 0));
    const ly = applyDeadzone(-(pad.axes[0] || 0));
    const az = applyDeadzone(-(pad.axes[2] || 0));
    stick.x = -ly;
    stick.y = -lx;
    drawJoystick();

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
    // D-pad
    if (pad.buttons[12]?.pressed) sendArm("x+");
    if (pad.buttons[13]?.pressed) sendArm("x-");
    if (pad.buttons[14]?.pressed) sendArm("z-");
    if (pad.buttons[15]?.pressed) sendArm("z+");
  }

  window.addEventListener("gamepadconnected", (e) => {
    gamepadIndex = e.gamepad.index;
    gamepadLabel.textContent = `Gamepad: ${e.gamepad.id}`;
  });
  window.addEventListener("gamepaddisconnected", () => {
    gamepadIndex = null;
    resetStick();
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
  }

  async function boot() {
    drawJoystick();
    const cfg = await fetch("/api/config").then((r) => r.json());
    speed = cfg.speed ?? speed;
    turn = cfg.turn ?? turn;
    speedEl.value = String(speed);
    turnEl.value = String(turn);

    const base = cfg.video_base || "http://localhost:8080";
    rawImg.src = streamUrl(base, cfg.topics.raw);

    if (cfg.sim) {
      modeEl.textContent = "SIM";
      layoutEl.classList.remove("tether");
      overviewPane.hidden = false;
      overviewImg.src = streamUrl(base, cfg.topics.overview);
    } else {
      modeEl.textContent = "TETHER";
      layoutEl.classList.add("tether");
      overviewPane.hidden = true;
    }

    connectWs();
    setInterval(pollGamepad, 1000 / SEND_HZ);
  }

  boot().catch((err) => {
    setStatus(`failed: ${err}`, false);
  });
})();
