"""Deterministic safety-policy values and decisions."""

from dataclasses import dataclass
from enum import StrEnum
from math import isfinite

from ros2_manipulator_mcp.domain import PrimitiveType, Vector3


class SafetySeverity(StrEnum):
    """Severity of one policy finding."""

    WARNING = "warning"
    ERROR = "error"


@dataclass(frozen=True)
class SafetyFinding:
    """A stable policy finding suitable for structured reporting."""

    code: str
    message: str
    severity: SafetySeverity


@dataclass(frozen=True)
class SafetyDecision:
    """Complete deterministic result of one policy evaluation."""

    policy_id: str
    findings: tuple[SafetyFinding, ...] = ()

    @property
    def allowed(self) -> bool:
        """Return whether no error finding blocks the operation."""
        return not any(
            finding.severity is SafetySeverity.ERROR
            for finding in self.findings
        )


@dataclass(frozen=True)
class WorkspaceBounds:
    """Inclusive Cartesian workspace bounds."""

    minimum: Vector3
    maximum: Vector3

    def __post_init__(self) -> None:
        if (
            self.minimum.x > self.maximum.x
            or self.minimum.y > self.maximum.y
            or self.minimum.z > self.maximum.z
        ):
            raise ValueError("workspace minimum must not exceed maximum")

    def contains(self, point: Vector3) -> bool:
        """Return whether a point lies inside all configured bounds."""
        return (
            self.minimum.x <= point.x <= self.maximum.x
            and self.minimum.y <= point.y <= self.maximum.y
            and self.minimum.z <= point.z <= self.maximum.z
        )


@dataclass(frozen=True)
class SafetyPolicy:
    """Configuration-driven limits for public v0.1.0 operations."""

    policy_id: str = "default-v1"
    allowed_planning_groups: tuple[str, ...] = ()
    max_planning_time_seconds: float = 10.0
    max_planning_attempts: int = 5
    max_velocity_scaling_factor: float = 1.0
    max_acceleration_scaling_factor: float = 1.0
    max_joint_targets: int = 16
    workspace: WorkspaceBounds | None = None
    max_cartesian_waypoints: int = 100
    min_cartesian_step: float = 0.0001
    max_cartesian_step: float = 0.1
    min_cartesian_completion_fraction: float = 0.95
    require_collision_avoidance: bool = True
    allowed_object_id_prefixes: tuple[str, ...] = ()
    max_collision_objects: int = 100
    max_primitive_dimension: float = 5.0
    allowed_primitive_types: tuple[PrimitiveType, ...] = tuple(PrimitiveType)
    allowed_scene_frames: tuple[str, ...] = ()
    allow_object_replace: bool = False
    require_scene_revision: bool = True

    def __post_init__(self) -> None:
        if not self.policy_id.strip():
            raise ValueError("policy_id must not be empty")
        positive_values = {
            "max_planning_time_seconds": self.max_planning_time_seconds,
            "max_velocity_scaling_factor": self.max_velocity_scaling_factor,
            "max_acceleration_scaling_factor": self.max_acceleration_scaling_factor,
            "min_cartesian_step": self.min_cartesian_step,
            "max_cartesian_step": self.max_cartesian_step,
            "min_cartesian_completion_fraction": (
                self.min_cartesian_completion_fraction
            ),
            "max_primitive_dimension": self.max_primitive_dimension,
        }
        if any(
            not isfinite(value) or value <= 0
            for value in positive_values.values()
        ):
            raise ValueError("policy numeric limits must be finite and positive")
        if self.max_velocity_scaling_factor > 1:
            raise ValueError("maximum velocity scaling must not exceed one")
        if self.max_acceleration_scaling_factor > 1:
            raise ValueError("maximum acceleration scaling must not exceed one")
        if self.min_cartesian_completion_fraction > 1:
            raise ValueError("minimum Cartesian completion must not exceed one")
        if self.min_cartesian_step > self.max_cartesian_step:
            raise ValueError("minimum Cartesian step must not exceed maximum")
        for name, value in (
            ("max_planning_attempts", self.max_planning_attempts),
            ("max_joint_targets", self.max_joint_targets),
            ("max_cartesian_waypoints", self.max_cartesian_waypoints),
            ("max_collision_objects", self.max_collision_objects),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        for name, values in (
            ("allowed_planning_groups", self.allowed_planning_groups),
            ("allowed_object_id_prefixes", self.allowed_object_id_prefixes),
            ("allowed_scene_frames", self.allowed_scene_frames),
        ):
            if any(not value.strip() for value in values):
                raise ValueError(f"{name} must not contain empty values")
        if any(
            not isinstance(value, PrimitiveType)
            for value in self.allowed_primitive_types
        ):
            raise ValueError("allowed_primitive_types contains an invalid type")
