#!/usr/bin/env bash
# Container entrypoint. Starts an in-container Xvfb when XVFB=1 so Gazebo's
# Ogre/GLX path can render camera sensors with no host display (Mac/Windows).
# This is not XQuartz — nothing is forwarded to the host.
set -euo pipefail

if [[ "${XVFB:-}" == "1" ]]; then
  display="${DISPLAY:-:99}"
  export DISPLAY="${display}"
  if ! pgrep -x Xvfb >/dev/null 2>&1; then
    # shellcheck disable=SC2086
    Xvfb "${display}" -screen 0 1280x1024x24 -ac +extension GLX +render -noreset \
      >/tmp/xvfb.log 2>&1 &
    # Give GLX a moment before anything (Gazebo) probes the display.
    sleep 0.5
  fi
fi

exec /ros_entrypoint.sh "$@"
