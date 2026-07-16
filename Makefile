# ---------------------------------------------------------------------------
# Host detection -> picks the right compose override automatically.
#   WSL2   : base + wsl2  (GPU + WSLg display, Gazebo GUI works)
#   Mac    : base + mac   (no GPU, no X, port-mapped networking, always headless)
#   Linux  : base only
#
# Mac has no X11 at all: video goes to the browser over HTTP instead (see
# bringup-camera). XQuartz used to be configured here on every make invocation,
# but it could never render anything anyway - GL has no working driver inside
# the emulated x86 image, so rqt died on swrast and Gazebo's GUI was hopeless.
# WSL2 keeps its display: WSLg works, and it's the only place the Gazebo GUI is
# actually usable.
# ---------------------------------------------------------------------------
UNAME_S := $(shell uname -s)
IS_WSL  := $(shell grep -qi microsoft /proc/version 2>/dev/null && echo 1)

ifeq ($(IS_WSL),1)
  PLATFORM      := wsl2
  COMPOSE_FILES := -f docker-compose.yml -f docker-compose.wsl2.yml
else ifeq ($(UNAME_S),Darwin)
  PLATFORM      := mac
  COMPOSE_FILES := -f docker-compose.yml -f docker-compose.mac.yml
  # Not a choice on Mac: there's no display to render into.
  HEADLESS := 1
else
  PLATFORM      := linux
  COMPOSE_FILES := -f docker-compose.yml
endif

DC   := docker compose $(COMPOSE_FILES)
EXEC := $(DC) exec robomaster-sim bash -c

# Video goes to the browser, not an X11 window — see bringup-camera.
RAW_URL  := http://localhost:8080/stream?topic=/camera/image_raw
TAGS_URL := http://localhost:8080/stream?topic=/camera/image_annotated
ifeq ($(UNAME_S),Darwin)
  OPEN := open
else
  OPEN := xdg-open
endif
SETUP := source /opt/ros/humble/setup.bash && cd /root/ros2_ws && [ -f install/setup.bash ] && source install/setup.bash;

.DEFAULT_GOAL := help
.PHONY: help image up down shell build test-gpu test-connection bringup \
        bringup-teleop bringup-camera bringup-detection rebuild clean

help: ## Show this help
	@echo "platform: $(PLATFORM)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2}'


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
# Each target is self-contained: it stands up exactly the subsystem it tests and
# nothing else, so a failure points at one thing. Don't chain them — pick one.
#
#   bringup            everything: drivetrain + camera + detection
#   bringup-teleop     drivetrain only, then hands you the keyboard
#   bringup-camera     camera only — is the camera alive?
#   bringup-detection  camera + detection — are tags being found?
#
# SIM in .env picks the backend (true = Gazebo, false = the physical robot).
# Video is watched in a browser at :8080 — there is no GUI window anywhere.
# HEADLESS=1 drops the Gazebo GUI; it's forced on Mac, which has no display.
LAUNCH = ros2 launch robomaster_bringup bringup.launch.py \
	  headless:=$(if $(filter 1,$(HEADLESS)),true,false)

bringup: build ## Everything: drivetrain + camera + detection
ifneq ($(IS_WSL),1)
	@echo "NOTE: no GPU passthrough on '$(PLATFORM)'. If SIM=true, expect Gazebo to be slow."
endif
	@echo "  camera: $(RAW_URL)"
	@echo "  tags:   $(TAGS_URL)"
	@echo "  drive:  make shell, then: ros2 run teleop_twist_keyboard \\"
	@echo "          teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop"
	$(EXEC) "$(SETUP) $(LAUNCH)"

# Self-contained: brings up the drivetrain in the background (no camera, no
# detection), waits for the mux, then hands you the keyboard in the foreground.
#
# The stack can't live in the same ros2 launch as teleop: teleop_twist_keyboard
# reads raw stdin, and a launch child process has no terminal, so it would never
# see a keypress. Hence background stack + foreground teleop, torn down together.
bringup-teleop: build ## Drivetrain only, then drive it with the keyboard
	@$(DC) exec -T robomaster-sim bash -c "pgrep -f '[b]ringup.launch.py' >/dev/null" \
	  && { echo ""; \
	       echo "  A bringup is already running in another terminal."; \
	       echo "  These targets are each a self-contained stack — stop that one first."; \
	       echo ""; exit 1; } || true
	@echo "  starting drivetrain (no camera)..."
	@$(DC) exec -d robomaster-sim bash -c "$(SETUP) \
	  $(LAUNCH) camera:=false detection:=false > /tmp/teleop_stack.log 2>&1"
	@$(DC) exec -T robomaster-sim bash -c "$(SETUP) \
	  for i in \$$(seq 1 60); do \
	    ros2 node list 2>/dev/null | grep -q cmd_vel_mux && exit 0; sleep 2; \
	  done; exit 1" \
	  || { echo "  drivetrain never came up — see /tmp/teleop_stack.log in the container"; \
	       $(DC) exec -T robomaster-sim tail -20 /tmp/teleop_stack.log; exit 1; }
	@echo "  ready — keys below actually drive it now."
	-$(DC) exec robomaster-sim bash -c "$(SETUP) \
	  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
	  --ros-args -r /cmd_vel:=/cmd_vel_teleop"
	@# [c] so pkill doesn't match this very command line and SIGTERM its own
	@# shell (that's what the stray "Error 143" was). Matches only the stack
	@# started above, never a bringup you have running elsewhere.
	-@$(DC) exec -T robomaster-sim bash -c "pkill -f '[c]amera:=false detection:=false' || true"
	@echo "  drivetrain stopped."

# Camera only: no controllers, no detection. Proves the feed is alive.
# On SIM=false the camera arms the video stream itself (no driver is holding the
# control port); on SIM=true Gazebo still comes up, just with no controllers.
bringup-camera: build ## Camera only — is the camera alive?
	@echo "  watch: $(RAW_URL)"
	@($(OPEN) "$(RAW_URL)" >/dev/null 2>&1 &) || true
	$(EXEC) "$(SETUP) $(LAUNCH) control:=false detection:=false"

# Camera + detection, no drivetrain: proves tags are found without a robot that
# can drive away.
bringup-detection: build ## Camera + AprilTag detection
	@echo "  watch: $(TAGS_URL)"
	@($(OPEN) "$(TAGS_URL)" >/dev/null 2>&1 &) || true
	$(EXEC) "$(SETUP) $(LAUNCH) control:=false"


# --- Maintenance ------------------------------------------------------------

rebuild: up ## Nuke build artifacts and rebuild the workspace clean
	$(EXEC) "cd /root/ros2_ws && rm -rf build install log && $(SETUP) colcon build --symlink-install"

clean: up ## Remove workspace build artifacts
	$(EXEC) "cd /root/ros2_ws && rm -rf build install log"