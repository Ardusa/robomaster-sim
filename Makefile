# ---------------------------------------------------------------------------
# Host detection -> picks the right compose override automatically.
#   WSL2   : base + wsl2  (GPU + WSLg display, full Gazebo)
#   Mac    : base + mac   (no GPU, XQuartz display, port-mapped networking)
#   Linux  : base only
# ---------------------------------------------------------------------------
UNAME_S := $(shell uname -s)
IS_WSL  := $(shell grep -qi microsoft /proc/version 2>/dev/null && echo 1)

ifeq ($(IS_WSL),1)
  PLATFORM      := wsl2
  COMPOSE_FILES := -f docker-compose.yml -f docker-compose.wsl2.yml
else ifeq ($(UNAME_S),Darwin)
  PLATFORM      := mac
  COMPOSE_FILES := -f docker-compose.yml -f docker-compose.mac.yml
else
  PLATFORM      := linux
  COMPOSE_FILES := -f docker-compose.yml
endif

DC   := docker compose $(COMPOSE_FILES)
EXEC := $(DC) exec robomaster-sim bash -c
# source ROS + workspace overlay before any ros2/colcon command
SETUP := source /opt/ros/humble/setup.bash && cd /root/ros2_ws && [ -f install/setup.bash ] && source install/setup.bash;

.DEFAULT_GOAL := help
.PHONY: help image up down shell check-gpu build bringup sim tether rebuild clean

help: ## Show this help
	@echo "platform: $(PLATFORM)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2}'

# --- container lifecycle (literal docker compose) --------------------------
image: ## Build the Docker image (rarely needed)
	$(DC) build

up: ## Start the container (detached)
	$(DC) up -d

down: ## Stop and remove the container
	$(DC) down

shell: up ## Open a bash shell in the container
	$(DC) exec robomaster-sim bash

check-gpu: up ## nvidia-smi inside the container (WSL2 only)
	$(DC) exec robomaster-sim nvidia-smi

# --- workspace build -------------------------------------------------------
build: up ## Colcon-build the ROS2 workspace
	$(EXEC) "$(SETUP) colcon build --symlink-install"

# --- ROS2 layer: shared bringup + two backends -----------------------------
bringup: build ## Launch shared ROS2 nodes only (headless, no sim/robot)
	$(EXEC) "$(SETUP) ros2 launch robomaster_gazebo bringup.launch.py"

sim: build ## Bringup + Gazebo (Windows/WSL2 GPU; runs but unusably slow on Mac)
ifneq ($(IS_WSL),1)
	@echo "WARNING: Gazebo has no GPU passthrough on '$(PLATFORM)'. Expect unusable performance."
endif
	$(EXEC) "$(SETUP) ros2 launch robomaster_gazebo sim.launch.py"

tether: build ## Bringup + physical RoboMaster driver (scaffolding: placeholder node)
	$(EXEC) "$(SETUP) ros2 launch robomaster_gazebo tether.launch.py"

# --- maintenance -----------------------------------------------------------
rebuild: up ## Nuke build artifacts and rebuild the workspace clean
	$(EXEC) "cd /root/ros2_ws && rm -rf build install log && $(SETUP) colcon build --symlink-install"

clean: up ## Remove workspace build artifacts
	$(EXEC) "cd /root/ros2_ws && rm -rf build install log"