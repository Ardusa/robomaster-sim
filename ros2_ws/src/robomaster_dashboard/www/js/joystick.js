/** Dual canvas sticks: interactive mecanum + read-only arm preview. */

const DEADZONE = 0.12;

export function applyDeadzone(v) {
  return Math.abs(v) < DEADZONE ? 0 : v;
}

function drawStick(canvas, stick, { knobColor = "#3d9cf0", dim = false } = {}) {
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const r = Math.min(w, h) * 0.42;
  ctx.clearRect(0, 0, w, h);
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.fillStyle = dim ? "#121820" : "#151b24";
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
  ctx.fillStyle = knobColor;
  ctx.globalAlpha = dim ? 0.65 : 1;
  ctx.fill();
  ctx.globalAlpha = 1;
}

export function createJoystick(canvas, { interactive = true } = {}) {
  const stick = { x: 0, y: 0 };
  let dragging = false;
  let locked = false; // gamepad owns the stick

  function redraw() {
    drawStick(canvas, stick, {
      knobColor: interactive ? "#3d9cf0" : "#8b97a8",
      dim: !interactive,
    });
  }

  function setFromPointer(clientX, clientY) {
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
    redraw();
  }

  function reset() {
    stick.x = 0;
    stick.y = 0;
    redraw();
  }

  function setValue(x, y) {
    stick.x = x;
    stick.y = y;
    redraw();
  }

  function setLocked(v) {
    locked = v;
  }

  if (interactive) {
    canvas.addEventListener("pointerdown", (e) => {
      if (locked) return;
      dragging = true;
      canvas.setPointerCapture(e.pointerId);
      setFromPointer(e.clientX, e.clientY);
    });
    canvas.addEventListener("pointermove", (e) => {
      if (!dragging || locked) return;
      setFromPointer(e.clientX, e.clientY);
    });
    function endDrag(e) {
      if (!dragging) return;
      dragging = false;
      try {
        canvas.releasePointerCapture(e.pointerId);
      } catch (_) {
        /* ignore */
      }
      if (!locked) reset();
    }
    canvas.addEventListener("pointerup", endDrag);
    canvas.addEventListener("pointercancel", endDrag);
  }

  redraw();
  return {
    stick,
    reset,
    setValue,
    setLocked,
    redraw,
  };
}
