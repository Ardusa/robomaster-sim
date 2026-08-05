# ---------------------------------------------------------------------------
# Host detection -> picks the right compose override automatically.
#   WSL2   : base + wsl2  (GPU + WSLg display, Gazebo GUI works)
#   Mac    : base + mac   (no GPU, no X, port-mapped networking, always headless)
#   Linux  : base only
# ---------------------------------------------------------------------------
ifeq ($(OS),Windows_NT)
	UNAME_S :=
	IS_WSL  :=
else
	UNAME_S := $(shell uname -s)
	IS_WSL  := $(shell grep -qi microsoft /proc/version 2>/dev/null && echo 1)
endif

ifeq ($(OS),Windows_NT)
	PLATFORM      := windows
	COMPOSE_FILES := -f docker-compose.yml
	HEADLESS := 1
else ifeq ($(IS_WSL),1)
	PLATFORM      := wsl2
	COMPOSE_FILES := -f docker-compose.yml -f docker-compose.wsl2.yml
else ifeq ($(UNAME_S),Darwin)
	PLATFORM      := mac
	COMPOSE_FILES := -f docker-compose.yml -f docker-compose.mac.yml
	HEADLESS := 1
else
	PLATFORM      := linux
	COMPOSE_FILES := -f docker-compose.yml
endif

DC   := docker compose $(COMPOSE_FILES)
EXEC := $(DC) exec robomaster-sim bash -c

RAW_URL  := http://localhost:8080/stream?topic=/camera/image_raw
TAGS_URL := http://localhost:8080/stream?topic=/camera/image_annotated
ifeq ($(UNAME_S),Darwin)
  OPEN := open
else
  OPEN := xdg-open
endif

ifeq ($(OS),Windows_NT)
	OPEN := start
endif
SETUP := source /opt/ros/humble/setup.bash && cd /root/ros2_ws && [ -f install/setup.bash ] && source install/setup.bash;

# Kill a bringup session inside the container. One physical line so make does
# not split it (multiline defines + CRLF previously produced "-docker").
# Meant for a recipe shell that already set $session (written $$session here).
BRINGUP_CLEANUP_SH = $(DC) exec -T robomaster-sim bash -lc "pkill -f \"rmbringup=$$session\" || true; pkill -f \"[r]os2 launch robomaster_bringup\" || true; pkill -f \"[w]eb_video_server\" || true; pkill -f \"[c]md_vel_mux.py\" || true; pkill -f \"[a]priltag\" || true; pkill -f \"[t]ag_overlay\" || true; pkill -f \"[r]ectify\" || true; pkill -f \"[r]os_gz_sim create\" || true; pkill -f \"[i]gn gazebo\" || true; pkill -f \"[s]dk_bridge_node\" || true; pkill -f \"[a]rm_node.py\" || true; pkill -f \"[t]eleop_node.py\" || true; true"

.DEFAULT_GOAL := help
.PHONY: help image up down shell build test-gpu test-connection bringup \
        bringup-teleop bringup-camera bringup-detection rebuild clean

help: ## Show this help
	@echo "platform: $(PLATFORM)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}'


# --- Container Lifecycle ----------------------------------------------------

image: ## Build the Docker image (rarely needed)
	$(DC) build

up: ## Start the container (detached)
	$(DC) up -d

down: ## Stop and remove the container
	$(DC) down

shell: up ## Open a bash shell in the container
	$(DC) exec robomaster-sim bash

build: up ## Colcon-build the ROS2 workspace
	$(EXEC) "$(SETUP) colcon build --symlink-install"


# --- Testing ---------------------------------------------------------------

test-gpu: up ## nvidia-smi inside the container (WSL2 only)
	$(DC) exec robomaster-sim nvidia-smi

test-connection: build ## Standalone TCP connectivity check against the real robot
	$(EXEC) "$(SETUP) ros2 run robomaster_driver connection_test"


# --- ROS2 Bringup -----------------------------------------------------------
# Process model (all of these):
#   1. Guard against an already-running bringup
#   2. Start the stack detached with a unique session marker
#   3. Wait for readiness
#   4. Foreground teleop when the target needs it (or wait for detection)
#   5. trap cleanup in the *same* shell (no recursive make)
#
# SIM in .env picks the backend (true = Gazebo, false = physical robot).
LAUNCH = ros2 launch robomaster_bringup bringup.launch.py \
	  headless:=$(if $(filter 1,$(HEADLESS)),true,false)

define bringup_guard
	@$(DC) exec -T robomaster-sim bash -c "pgrep -f '[r]mbringup=' >/dev/null" \
	  && { echo ""; \
	       echo "  A bringup session is already running."; \
	       echo "  Stop it (Ctrl-C in that terminal) before starting another."; \
	       echo ""; exit 1; } || true
endef

bringup: build ## Full stack (bg) + teleop (fg)
ifneq ($(IS_WSL),1)
	@echo "NOTE: no GPU passthrough on '$(PLATFORM)'. If SIM=true, expect Gazebo to be slow."
endif
	$(bringup_guard)
	@session="rmbringup-$$$$-$$(date +%s)"; \
	echo "  session: $$session"; \
	echo "  camera: $(RAW_URL)"; \
	echo "  tags:   $(TAGS_URL)"; \
	cleanup() { $(BRINGUP_CLEANUP_SH); echo "  bringup session stopped."; }; \
	trap cleanup EXIT INT TERM; \
	$(DC) exec -d robomaster-sim bash -c ": rmbringup=$$session; $(SETUP) \
	  $(LAUNCH) control:=true arm:=true camera:=true detection:=true \
	  video_server:=true > /tmp/bringup_stack.log 2>&1"; \
	$(DC) exec -T robomaster-sim bash -c "$(SETUP) \
	  for i in \$$(seq 1 90); do \
	    ros2 node list 2>/dev/null | grep -q cmd_vel_mux && \
	    ros2 node list 2>/dev/null | grep -q robomaster_arm && exit 0; \
	    sleep 2; \
	  done; exit 1" \
	  || { echo "  stack never came up — see /tmp/bringup_stack.log"; \
	       $(DC) exec -T robomaster-sim tail -40 /tmp/bringup_stack.log; exit 1; }; \
	echo "  ready — teleop in foreground (Ctrl-C tears down the stack)."; \
	$(DC) exec robomaster-sim bash -c "$(SETUP) \
	  ros2 run robomaster_teleop teleop_node.py" || true

bringup-teleop: build ## Drivetrain + arm (bg) + teleop (fg)
	$(bringup_guard)
	@session="rmbringup-$$$$-$$(date +%s)"; \
	echo "  session: $$session"; \
	echo "  starting drivetrain + arm..."; \
	cleanup() { $(BRINGUP_CLEANUP_SH); echo "  bringup session stopped."; }; \
	trap cleanup EXIT INT TERM; \
	$(DC) exec -d robomaster-sim bash -c ": rmbringup=$$session; $(SETUP) \
	  $(LAUNCH) control:=true arm:=true camera:=false detection:=false \
	  video_server:=false > /tmp/teleop_stack.log 2>&1"; \
	$(DC) exec -T robomaster-sim bash -c "$(SETUP) \
	  for i in \$$(seq 1 90); do \
	    ros2 node list 2>/dev/null | grep -q cmd_vel_mux && \
	    ros2 node list 2>/dev/null | grep -q robomaster_arm && exit 0; \
	    sleep 2; \
	  done; exit 1" \
	  || { echo "  stack never came up — see /tmp/teleop_stack.log"; \
	       $(DC) exec -T robomaster-sim tail -40 /tmp/teleop_stack.log; exit 1; }; \
	echo "  ready — teleop in foreground (Ctrl-C tears down the stack)."; \
	$(DC) exec robomaster-sim bash -c "$(SETUP) \
	  ros2 run robomaster_teleop teleop_node.py" || true

bringup-detection: build ## Camera + AprilTag detection (no teleop)
	$(bringup_guard)
	@session="rmbringup-$$$$-$$(date +%s)"; \
	echo "  session: $$session"; \
	echo "  watch: $(TAGS_URL)"; \
	($(OPEN) "$(TAGS_URL)" >/dev/null 2>&1 &) || true; \
	cleanup() { $(BRINGUP_CLEANUP_SH); echo "  bringup session stopped."; }; \
	trap cleanup EXIT INT TERM; \
	$(DC) exec -d robomaster-sim bash -c ": rmbringup=$$session; $(SETUP) \
	  $(LAUNCH) control:=false arm:=false camera:=true detection:=true \
	  video_server:=true > /tmp/detection_stack.log 2>&1"; \
	$(DC) exec -T robomaster-sim bash -c "$(SETUP) \
	  for i in \$$(seq 1 90); do \
	    ros2 node list 2>/dev/null | grep -qE 'apriltag|web_video_server' && exit 0; \
	    sleep 2; \
	  done; exit 1" \
	  || { echo "  stack never came up — see /tmp/detection_stack.log"; \
	       $(DC) exec -T robomaster-sim tail -40 /tmp/detection_stack.log; exit 1; }; \
	echo "  detection stack up — Ctrl-C to stop."; \
	$(DC) exec robomaster-sim bash -c "tail -f /tmp/detection_stack.log" || true

bringup-camera: build ## Camera only — is the camera alive?
	$(bringup_guard)
	@session="rmbringup-$$$$-$$(date +%s)"; \
	echo "  session: $$session"; \
	echo "  watch: $(RAW_URL)"; \
	($(OPEN) "$(RAW_URL)" >/dev/null 2>&1 &) || true; \
	cleanup() { $(BRINGUP_CLEANUP_SH); echo "  bringup session stopped."; }; \
	trap cleanup EXIT INT TERM; \
	$(DC) exec -d robomaster-sim bash -c ": rmbringup=$$session; $(SETUP) \
	  $(LAUNCH) control:=false arm:=false camera:=true detection:=false \
	  video_server:=true > /tmp/camera_stack.log 2>&1"; \
	$(DC) exec -T robomaster-sim bash -c "$(SETUP) \
	  for i in \$$(seq 1 90); do \
	    ros2 node list 2>/dev/null | grep -qE 'camera|web_video_server' && exit 0; \
	    sleep 2; \
	  done; exit 1" \
	  || { echo "  stack never came up — see /tmp/camera_stack.log"; \
	       $(DC) exec -T robomaster-sim tail -40 /tmp/camera_stack.log; exit 1; }; \
	echo "  camera stack up — Ctrl-C to stop."; \
	$(DC) exec robomaster-sim bash -c "tail -f /tmp/camera_stack.log" || true


# --- Maintenance ------------------------------------------------------------

rebuild: up ## Nuke build artifacts and rebuild the workspace clean
	$(EXEC) "cd /root/ros2_ws && rm -rf build install log && $(SETUP) colcon build --symlink-install"

clean: up ## Remove workspace build artifacts
	$(EXEC) "cd /root/ros2_ws && rm -rf build install log"
