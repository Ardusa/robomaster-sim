/** Live arm / gripper / fidelity readouts from WS state frames. */

const GRIPPER_LABELS = {
  0: "unknown",
  1: "open",
  2: "closed",
};

export function createTelemetry(els) {
  const {
    armX,
    armZ,
    armMoving,
    gripperLabel,
    fidelityBadge,
    gotoX,
    gotoZ,
  } = els;

  let seededGoto = false;

  function onState(state) {
    if (state.arm) {
      armX.textContent = state.arm.x.toFixed(3);
      armZ.textContent = state.arm.z.toFixed(3);
      armMoving.hidden = !state.arm.moving;
      if (!seededGoto && gotoX && gotoZ) {
        gotoX.value = state.arm.x.toFixed(3);
        gotoZ.value = state.arm.z.toFixed(3);
        seededGoto = true;
      }
    }

    if (state.gripper && gripperLabel) {
      const name = GRIPPER_LABELS[state.gripper.state] ?? "unknown";
      const opening = Number.isFinite(state.gripper.opening)
        ? ` (${state.gripper.opening.toFixed(3)} m)`
        : "";
      gripperLabel.textContent = `Gripper: ${name}${opening}`;
    }

    if (fidelityBadge) {
      const fidelity = state.fidelity || (state.sim ? "ground_truth" : "estimated");
      fidelityBadge.hidden = false;
      fidelityBadge.classList.remove("truth", "estimated");
      if (fidelity === "ground_truth") {
        fidelityBadge.textContent = "Ground truth";
        fidelityBadge.classList.add("truth");
      } else {
        fidelityBadge.textContent = "Estimated";
        fidelityBadge.classList.add("estimated");
      }
    }
  }

  return { onState };
}
