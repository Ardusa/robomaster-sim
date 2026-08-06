/** Natural-language command panel: submit + sequence/result display. */

function formatAction(action) {
  const type = action.type || "?";
  if (type === "navigate") {
    if (action.target_zone) {
      return `navigate → ${action.target_zone}`;
    }
    if (action.use_explicit_pose) {
      return (
        `navigate → (${Number(action.x).toFixed(2)}, ` +
        `${Number(action.y).toFixed(2)}, θ=${Number(action.theta).toFixed(2)})`
      );
    }
    return "navigate → (unset)";
  }
  if (type === "arm_goto") {
    return (
      `arm_goto → x=${Number(action.arm_x).toFixed(3)}, ` +
      `z=${Number(action.arm_z).toFixed(3)}`
    );
  }
  if (type === "gripper") {
    return `gripper → ${action.gripper_open ? "open" : "close"}`;
  }
  return type;
}

export function createCommandPanel({ form, input, sequenceEl, resultEl, send }) {
  form.addEventListener("submit", (e) => {
    e.preventDefault();
    const text = (input.value || "").trim();
    if (!text) return;
    sequenceEl.innerHTML = "";
    resultEl.textContent = "running…";
    resultEl.classList.remove("ok", "bad");
    send({ type: "command", text });
    input.value = "";
  });

  function onActionSequence(msg) {
    sequenceEl.innerHTML = "";
    const actions = msg.actions || [];
    if (!actions.length) {
      const li = document.createElement("li");
      li.textContent = "(empty sequence)";
      sequenceEl.appendChild(li);
      return;
    }
    for (const action of actions) {
      const li = document.createElement("li");
      li.textContent = formatAction(action);
      sequenceEl.appendChild(li);
    }
  }

  function onCommandResult(msg) {
    const ok = Boolean(msg.success);
    resultEl.textContent = ok
      ? `ok — ${msg.message || "done"}`
      : `failed — ${msg.message || "unknown error"}`;
    resultEl.classList.toggle("ok", ok);
    resultEl.classList.toggle("bad", !ok);
  }

  return { onActionSequence, onCommandResult };
}
