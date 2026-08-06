/** Arm setpoint panel: presets, numeric goto, gripper buttons. */

export function createArmPanel({ presetGrid, gotoForm, gotoX, gotoZ, send }) {
  function buildPresets(presets) {
    presetGrid.innerHTML = "";
    for (const [name, pose] of Object.entries(presets || {})) {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.dataset.arm = `preset_${name}`;
      const title = name.charAt(0).toUpperCase() + name.slice(1);
      btn.innerHTML = `${title}<span class="coord">${pose.x.toFixed(3)}, ${pose.z.toFixed(3)}</span>`;
      btn.addEventListener("click", () => send({ type: "arm", action: btn.dataset.arm }));
      presetGrid.appendChild(btn);
    }
  }

  function applyLimits(limits) {
    if (!limits) return;
    if (limits.x_min != null) gotoX.min = limits.x_min;
    if (limits.x_max != null) gotoX.max = limits.x_max;
    if (limits.z_min != null) gotoZ.min = limits.z_min;
    if (limits.z_max != null) gotoZ.max = limits.z_max;
  }

  gotoForm.addEventListener("submit", (e) => {
    e.preventDefault();
    const x = Number(gotoX.value);
    const z = Number(gotoZ.value);
    if (!Number.isFinite(x) || !Number.isFinite(z)) return;
    send({ type: "arm_goto", x, z });
  });

  document.querySelectorAll("[data-arm]").forEach((btn) => {
    // Preset buttons are built dynamically; only wire static gripper buttons here.
    if (btn.dataset.arm.startsWith("preset_")) return;
    btn.addEventListener("click", () => send({ type: "arm", action: btn.dataset.arm }));
  });

  return { buildPresets, applyLimits };
}
