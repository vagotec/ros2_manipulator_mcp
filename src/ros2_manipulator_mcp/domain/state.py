"""Manipulator joint and robot state."""

from dataclasses import dataclass

from ros2_manipulator_mcp.domain._validation import (
    finite_values,
    identifiers,
    non_negative,
)


@dataclass(frozen=True)
class JointState:
    """Ordered positions with optional velocity and effort samples."""

    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...] | None = None
    efforts: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        names = identifiers(self.joint_names, "joint_names")
        positions = finite_values(self.positions, "positions")
        if len(names) != len(positions):
            raise ValueError("joint_names and positions must have equal lengths")

        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "positions", positions)
        for field_name in ("velocities", "efforts"):
            values = getattr(self, field_name)
            if values is not None:
                validated = finite_values(values, field_name)
                if len(validated) != len(names):
                    raise ValueError(
                        f"joint_names and {field_name} must have equal lengths"
                    )
                object.__setattr__(self, field_name, validated)


@dataclass(frozen=True)
class RobotState:
    """A joint state with an optional non-negative sample time."""

    joints: JointState
    timestamp_seconds: float | None = None

    def __post_init__(self) -> None:
        if self.timestamp_seconds is not None:
            object.__setattr__(
                self,
                "timestamp_seconds",
                non_negative(self.timestamp_seconds, "timestamp_seconds"),
            )
