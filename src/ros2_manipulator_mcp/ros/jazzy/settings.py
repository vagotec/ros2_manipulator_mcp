"""Verified Jazzy MoveIt endpoint and timeout settings."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class JazzyMoveItSettings:
    """Names resolved for a root-namespace MoveIt `move_group` node."""

    node_name: str = "ros2_manipulator_mcp"
    joint_states_topic: str = "/joint_states"
    ik_service: str = "/compute_ik"
    fk_service: str = "/compute_fk"
    state_validity_service: str = "/check_state_validity"
    cartesian_path_service: str = "/compute_cartesian_path"
    motion_plan_service: str = "/plan_kinematic_path"
    get_planning_scene_service: str = "/get_planning_scene"
    apply_planning_scene_service: str = "/apply_planning_scene"
    execute_trajectory_action: str = "/execute_trajectory"
    trajectory_execution_event_topic: str = "/trajectory_execution_event"
    service_timeout_seconds: float = 5.0
    state_timeout_seconds: float = 2.0
    ik_timeout_seconds: float = 1.0
    joint_goal_tolerance: float = 0.0001

    def __post_init__(self) -> None:
        for field_name in (
            "node_name",
            "joint_states_topic",
            "ik_service",
            "fk_service",
            "state_validity_service",
            "cartesian_path_service",
            "motion_plan_service",
            "get_planning_scene_service",
            "apply_planning_scene_service",
            "execute_trajectory_action",
            "trajectory_execution_event_topic",
        ):
            if not getattr(self, field_name).strip():
                raise ValueError(f"{field_name} must not be empty")
        for field_name in (
            "service_timeout_seconds",
            "state_timeout_seconds",
            "ik_timeout_seconds",
            "joint_goal_tolerance",
        ):
            value = getattr(self, field_name)
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{field_name} must be finite and positive")
