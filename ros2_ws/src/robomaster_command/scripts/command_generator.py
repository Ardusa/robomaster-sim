from robomaster_command.msg import ActionPrimitive


def generate_action_sequence(prompt: str) -> list[ActionPrimitive]:
    return [
        ActionPrimitive(type="navigate", target_zone="hall_console"),
        ActionPrimitive(type="arm_goto", arm_x=0.15, arm_z=0.10),
        ActionPrimitive(type="gripper", gripper_open=False),
    ]
