import logging

from robomaster_command.msg import ActionPrimitive

logger = logging.getLogger(__name__)

_FALLBACK_SEQUENCE = [
    ActionPrimitive(type="navigate", target_zone="kitchen"),
    ActionPrimitive(type="arm_goto", arm_x=0.18, arm_z=0.12),
    ActionPrimitive(type="gripper", gripper_open=False),
]


def _find_matching_zone(
    prompt: str, zones: dict[str, dict[str, float]]
) -> str | None:
    prompt_lower = prompt.lower()
    matches: list[str] = []
    for zone in zones:
        zone_lower = zone.lower()
        forms = {zone_lower, zone_lower.replace("_", " ")}
        if any(form in prompt_lower for form in forms):
            matches.append(zone)
    if not matches:
        return None
    return max(matches, key=len)


# TODO: replace real-match branch with an Anthropic API call once ready; the
# restricted-zone refusal and the fallback path both stay as-is.
def generate_action_sequence(
    prompt: str,
    zones: dict[str, dict[str, float]],
    restricted: list[str],
) -> list[ActionPrimitive]:
    zone = _find_matching_zone(prompt, zones)
    if zone is None:
        logger.warning(
            "no zone matched prompt %r; using fallback sequence", prompt
        )
        return list(_FALLBACK_SEQUENCE)

    if zone in restricted:
        raise ValueError(f"target zone '{zone}' is restricted")

    return [
        ActionPrimitive(type="navigate", target_zone=zone),
        ActionPrimitive(type="arm_goto", arm_x=0.15, arm_z=0.10),
        ActionPrimitive(type="gripper", gripper_open=False),
    ]
