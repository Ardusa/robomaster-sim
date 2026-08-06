# Discrete action in a command sequence. Only the fields relevant to `type`
# are read; translator_node ignores the rest.
string type          # "navigate" | "arm_goto" | "gripper"

# navigate: resolved through config/semantic_maps/<world>.yaml by target_zone.
# If target_zone is empty, x/y/theta are used directly as a map-frame goal.
string target_zone
float64 x
float64 y
float64 theta

# arm_goto: Cartesian goal in arm_base_link frame, metres. Same convention as
# robomaster_arm/action/MoveArm.
float64 arm_x
float64 arm_z

# gripper
bool gripper_open
