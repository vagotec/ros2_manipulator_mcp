"""Immutable joint trajectories."""

from dataclasses import dataclass

from ros2_manipulator_mcp.domain._validation import (
    finite_values,
    identifiers,
    non_negative,
)


@dataclass(frozen=True)
class TrajectoryPoint:
    """One trajectory sample at a non-negative offset."""

    positions: tuple[float, ...]
    time_from_start_seconds: float
    velocities: tuple[float, ...] | None = None
    accelerations: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        positions = finite_values(self.positions, "positions")
        if not positions:
            raise ValueError("positions must not be empty")
        object.__setattr__(self, "positions", positions)
        object.__setattr__(
            self,
            "time_from_start_seconds",
            non_negative(self.time_from_start_seconds, "time_from_start_seconds"),
        )
        for field_name in ("velocities", "accelerations"):
            values = getattr(self, field_name)
            if values is not None:
                object.__setattr__(
                    self,
                    field_name,
                    finite_values(values, field_name),
                )


@dataclass(frozen=True)
class Trajectory:
    """An immutable trajectory with strictly increasing sample times."""

    joint_names: tuple[str, ...]
    points: tuple[TrajectoryPoint, ...]

    def __post_init__(self) -> None:
        names = identifiers(self.joint_names, "joint_names")
        points = tuple(self.points)
        if not points:
            raise ValueError("trajectory points must not be empty")

        previous_time = -1.0
        for point in points:
            if len(point.positions) != len(names):
                raise ValueError("trajectory positions must match joint_names")
            for field_name in ("velocities", "accelerations"):
                values = getattr(point, field_name)
                if values is not None and len(values) != len(names):
                    raise ValueError(
                        f"trajectory {field_name} must match joint_names"
                    )
            if point.time_from_start_seconds <= previous_time:
                raise ValueError("trajectory timestamps must strictly increase")
            previous_time = point.time_from_start_seconds

        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "points", points)
