"""Motion and Cartesian path planning models."""

from dataclasses import dataclass

from ros2_manipulator_mcp.domain._validation import (
    identifier,
    non_negative,
    positive,
    scaling_factor,
)
from ros2_manipulator_mcp.domain.geometry import Pose
from ros2_manipulator_mcp.domain.goals import JointGoal, PoseGoal
from ros2_manipulator_mcp.domain.state import RobotState
from ros2_manipulator_mcp.domain.trajectory import Trajectory


@dataclass(frozen=True)
class PlanningRequest:
    """A bounded request to plan to a joint or pose goal."""

    goal: JointGoal | PoseGoal
    start_state: RobotState | None = None
    max_velocity_scaling_factor: float = 1.0
    max_acceleration_scaling_factor: float = 1.0
    allowed_planning_time_seconds: float = 5.0
    planning_attempts: int = 1

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "max_velocity_scaling_factor",
            scaling_factor(
                self.max_velocity_scaling_factor,
                "max_velocity_scaling_factor",
            ),
        )
        object.__setattr__(
            self,
            "max_acceleration_scaling_factor",
            scaling_factor(
                self.max_acceleration_scaling_factor,
                "max_acceleration_scaling_factor",
            ),
        )
        object.__setattr__(
            self,
            "allowed_planning_time_seconds",
            positive(
                self.allowed_planning_time_seconds,
                "allowed_planning_time_seconds",
            ),
        )
        if (
            not isinstance(self.planning_attempts, int)
            or isinstance(self.planning_attempts, bool)
            or self.planning_attempts < 1
        ):
            raise ValueError("planning_attempts must be a positive integer")


@dataclass(frozen=True)
class MotionPlan:
    """An immutable stored motion plan and its provenance metadata."""

    plan_id: str
    request: PlanningRequest
    trajectory: Trajectory
    planning_duration_seconds: float
    scene_revision: str

    def __post_init__(self) -> None:
        identifier(self.plan_id, "plan_id")
        identifier(self.scene_revision, "scene_revision")
        object.__setattr__(
            self,
            "planning_duration_seconds",
            non_negative(
                self.planning_duration_seconds,
                "planning_duration_seconds",
            ),
        )


@dataclass(frozen=True)
class CartesianPathRequest:
    """A collision-aware Cartesian waypoint request."""

    group_name: str
    target_link: str
    waypoints: tuple[Pose, ...]
    max_step: float
    jump_threshold: float = 0.0
    avoid_collisions: bool = True
    start_state: RobotState | None = None
    max_velocity_scaling_factor: float = 1.0
    max_acceleration_scaling_factor: float = 1.0

    def __post_init__(self) -> None:
        identifier(self.group_name, "group_name")
        identifier(self.target_link, "target_link")
        waypoints = tuple(self.waypoints)
        if not waypoints:
            raise ValueError("waypoints must not be empty")
        object.__setattr__(self, "waypoints", waypoints)
        object.__setattr__(self, "max_step", positive(self.max_step, "max_step"))
        object.__setattr__(
            self,
            "jump_threshold",
            non_negative(self.jump_threshold, "jump_threshold"),
        )
        for field_name in (
            "max_velocity_scaling_factor",
            "max_acceleration_scaling_factor",
        ):
            object.__setattr__(
                self,
                field_name,
                scaling_factor(getattr(self, field_name), field_name),
            )


@dataclass(frozen=True)
class CartesianPathResult:
    """A Cartesian path fraction and its optional joint trajectory."""

    fraction: float
    trajectory: Trajectory | None

    def __post_init__(self) -> None:
        fraction = non_negative(self.fraction, "fraction")
        if fraction > 1:
            raise ValueError("fraction must be within [0, 1]")
        if fraction > 0 and self.trajectory is None:
            raise ValueError("a positive fraction requires a trajectory")
        object.__setattr__(self, "fraction", fraction)
