# ---------------------------------------------------------------------------
# Host detection -> picks the right compose override automatically.
#   WSL2   : base + wsl2  (GPU + WSLg display; GUI=true in .env is possible)
#   Mac    : base + mac   (no GPU, no X, port-mapped networking, always headless)
#   Linux  : base only
#
# Backend (SIM / WORLD / ROBOMASTER_IP) is controlled in .env, not Make flags.
# Gazebo runs headless by default — the dashboard on :8090 is the viewport.
# Set GUI=true in .env for the real Gazebo window (WSL2/Linux only).
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

RAW_URL        := http://localhost:8080/stream?topic=/camera/image_raw
ANNOTATED_URL  := http://localhost:8080/stream?topic=/camera/image_annotated
DASHBOARD_URL  := http://localhost:8090
ifeq ($(UNAME_S),Darwin)
  OPEN_CMD := open
else ifeq ($(OS),Windows_NT)
  OPEN_CMD := start
else
  OPEN_CMD := xdg-open
endif

SETUP := source /opt/ros/humble/setup.bash && cd /root/ros2_ws && [ -f install/setup.bash ] && source install/setup.bash;

# Session orchestration lives in scripts/bringup.sh (profiles: full|teleop|…).
BRINGUP := \
	DC="$(DC)" SETUP="$(SETUP)" HEADLESS="$(HEADLESS)" \
	RAW_URL="$(RAW_URL)" ANNOTATED_URL="$(ANNOTATED_URL)" DASHBOARD_URL="$(DASHBOARD_URL)" \
	OPEN_CMD="$(OPEN_CMD)" \
	bash scripts/bringup.sh

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
# Profiles are capability slices. SIM/WORLD/ROBOMASTER_IP come from .env.

bringup: build ## Full stack (bg) + dashboard (Ctrl-C tears down)
ifneq ($(IS_WSL),1)
	@echo "NOTE: no GPU passthrough on '$(PLATFORM)'. If SIM=true, expect Gazebo to be slow."
endif
	@$(BRINGUP) full

bringup-teleop: build ## Drivetrain + arm (bg) + keyboard teleop (fg fallback)
	@$(BRINGUP) teleop

bringup-detection: build ## Camera + COCO object detection (no teleop)
	@$(BRINGUP) detection

bringup-camera: build ## Camera only — is the camera alive?
	@$(BRINGUP) camera


# --- Maintenance ------------------------------------------------------------

rebuild: up ## Nuke build artifacts and rebuild the workspace clean
	$(EXEC) "cd /root/ros2_ws && rm -rf build install log && $(SETUP) colcon build --symlink-install"

clean: up ## Remove workspace build artifacts
	$(EXEC) "cd /root/ros2_ws && rm -rf build install log"
