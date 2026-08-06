#!/usr/bin/env bash
# Host-side bringup session machine. Invoked by the Makefile with DC/SETUP/etc.
# exported. Backend (SIM/WORLD/ROBOMASTER_IP) comes from .env via compose — not
# from Make flags.
set -euo pipefail

profile="${1:?usage: bringup.sh <full|teleop|camera|detection>}"

: "${DC:?DC (docker compose …) must be set}"
: "${SETUP:?SETUP must be set}"

RAW_URL="${RAW_URL:-http://localhost:8080/stream?topic=/camera/image_raw}"
ANNOTATED_URL="${ANNOTATED_URL:-${TAGS_URL:-http://localhost:8080/stream?topic=/camera/image_annotated}}"
DASHBOARD_URL="${DASHBOARD_URL:-http://localhost:8090}"
OPEN_CMD="${OPEN_CMD:-}"
HEADLESS="${HEADLESS:-}"

# Headless is decided in-container from $GUI in .env (bringup.launch.py).
# HEADLESS=1 is a platform force from the Makefile (Mac/Windows: no display
# path exists), which must win over GUI=true.
LAUNCH="ros2 launch robomaster_bringup bringup.launch.py"
if [[ "${HEADLESS}" == "1" ]]; then
  LAUNCH+=" headless:=true"
fi

if $DC exec -T robomaster-sim bash -c "pgrep -f '[r]mbringup=' >/dev/null"; then
  echo ""
  echo "  A bringup session is already running."
  echo "  Stop it (Ctrl-C in that terminal) before starting another."
  echo ""
  exit 1
fi

session="rmbringup-$$-$(date +%s)"
echo "  session: ${session}"

cleanup() {
  $DC exec -T robomaster-sim bash -lc \
    "pkill -f \"rmbringup=${session}\" || true; \
     pkill -f \"[r]os2 launch robomaster_bringup\" || true; \
     pkill -f \"[w]eb_video_server\" || true; \
     pkill -f \"[c]md_vel_mux.py\" || true; \
     pkill -f \"[o]bject_detector\" || true; \
     pkill -f \"[d]etection_overlay\" || true; \
     pkill -f \"[r]os_gz_sim create\" || true; \
     pkill -f \"[i]gn gazebo\" || true; \
     pkill -f \"[s]dk_bridge_node\" || true; \
     pkill -f \"[a]rm_node.py\" || true; \
     pkill -f \"[t]eleop_node.py\" || true; \
     pkill -f \"[d]ashboard_node\" || true; \
     pkill -f \"[g]rounding_node.py\" || true; \
     pkill -f \"[t]ranslator_node.py\" || true; \
     true"
  echo "  bringup session stopped."
}
trap cleanup EXIT INT TERM

# Wait until every grep pattern matches some ros2 node name (AND).
wait_ready() {
  local log="$1"
  shift
  local cond=""
  local pat
  for pat in "$@"; do
    if [[ -n "${cond}" ]]; then
      cond+=" && "
    fi
    cond+="ros2 node list 2>/dev/null | grep -qE \"${pat}\""
  done
  $DC exec -T robomaster-sim bash -c "${SETUP} \
    for i in \$(seq 1 90); do \
      if ${cond}; then exit 0; fi; \
      sleep 2; \
    done; exit 1" \
    || {
      echo "  stack never came up — see ${log}"
      $DC exec -T robomaster-sim tail -40 "${log}"
      exit 1
    }
}

open_url() {
  local url="$1"
  if [[ -n "${OPEN_CMD}" ]]; then
    ("${OPEN_CMD}" "${url}" >/dev/null 2>&1 &) || true
  fi
}

case "${profile}" in
  full)
    echo "  dashboard: ${DASHBOARD_URL}"
    echo "  camera:    ${RAW_URL}"
    echo "  detect:    ${ANNOTATED_URL}"
    $DC exec -d robomaster-sim bash -c ": rmbringup=${session}; ${SETUP} \
      ${LAUNCH} control:=true arm:=true camera:=true detection:=true \
      video_server:=true dashboard:=true command:=true > /tmp/bringup_stack.log 2>&1"
    wait_ready /tmp/bringup_stack.log 'cmd_vel_mux' 'robomaster_arm' 'robomaster_dashboard' 'robomaster_command_grounding'
    echo "  ready — open the dashboard (Ctrl-C tears down the stack)."
    open_url "${DASHBOARD_URL}"
    $DC exec robomaster-sim bash -c "tail -f /tmp/bringup_stack.log" || true
    ;;
  teleop)
    echo "  starting drivetrain + arm (keyboard teleop fallback)..."
    $DC exec -d robomaster-sim bash -c ": rmbringup=${session}; ${SETUP} \
      ${LAUNCH} control:=true arm:=true camera:=false detection:=false \
      video_server:=false dashboard:=false > /tmp/teleop_stack.log 2>&1"
    wait_ready /tmp/teleop_stack.log 'cmd_vel_mux' 'robomaster_arm'
    echo "  ready — teleop in foreground (Ctrl-C tears down the stack)."
    $DC exec robomaster-sim bash -c "${SETUP} \
      ros2 run robomaster_teleop teleop_node.py" || true
    ;;
  detection)
    echo "  watch: ${ANNOTATED_URL}"
    open_url "${ANNOTATED_URL}"
    $DC exec -d robomaster-sim bash -c ": rmbringup=${session}; ${SETUP} \
      ${LAUNCH} control:=false arm:=false camera:=true detection:=true \
      video_server:=true dashboard:=false > /tmp/detection_stack.log 2>&1"
    wait_ready /tmp/detection_stack.log 'object_detector' 'web_video_server'
    echo "  detection stack up — Ctrl-C to stop."
    $DC exec robomaster-sim bash -c "tail -f /tmp/detection_stack.log" || true
    ;;
  camera)
    echo "  watch: ${RAW_URL}"
    open_url "${RAW_URL}"
    $DC exec -d robomaster-sim bash -c ": rmbringup=${session}; ${SETUP} \
      ${LAUNCH} control:=false arm:=false camera:=true detection:=false \
      video_server:=true dashboard:=false > /tmp/camera_stack.log 2>&1"
    wait_ready /tmp/camera_stack.log 'camera|web_video_server'
    echo "  camera stack up — Ctrl-C to stop."
    $DC exec robomaster-sim bash -c "tail -f /tmp/camera_stack.log" || true
    ;;
  *)
    echo "unknown profile: ${profile}" >&2
    exit 1
    ;;
esac
