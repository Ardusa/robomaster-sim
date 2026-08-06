# Discrete action in a command sequence. Only the fields relevant to `type`
# are read; translator_node ignores the rest.
string type          # "navigate" | "arm_goto" | "gripper"

# navigate: resolved through config/semantic_maps/<world>.yaml by target_zone.
# If target_zone is empty and use_explicit_pose is true, x/y/theta are the
# map-frame goal. If neither is set, translator_node fails the step.
string target_zone
float64 x
float64 y
float64 theta
bool use_explicit_pose false

# arm_goto: Cartesian goal in arm_base_link frame, metres. Same convention as
# robomaster_arm/action/MoveArm.
float64 arm_x
float64 arm_z

# gripper
bool gripper_open
