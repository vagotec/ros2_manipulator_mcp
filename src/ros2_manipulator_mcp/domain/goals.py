"""Joint-space and Cartesian planning goals."""

from dataclasses import dataclass

from ros2_manipulator_mcp.domain._validation import (
    finite_values,
    identifier,
    identifiers,
    positive,
)
from ros2_manipulator_mcp.domain.geometry import Pose


@dataclass(frozen=True)
class JointGoal:
    """Joint targets for one planning group."""

    group_name: str
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]

    def __post_init__(self) -> None:
        identifier(self.group_name, "group_name")
        names = identifiers(self.joint_names, "joint_names")
        positions = finite_values(self.positions, "positions")
        if len(names) != len(positions):
            raise ValueError("joint_names and positions must have equal lengths")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "positions", positions)


@dataclass(frozen=True)
class PoseGoal:
    """A framed target pose for one link and planning group."""

    group_name: str
    target_link: str
    pose: Pose
    position_tolerance: float = 0.001
    orientation_tolerance: float = 0.01

    def __post_init__(self) -> None:
        identifier(self.group_name, "group_name")
        identifier(self.target_link, "target_link")
        object.__setattr__(
            self,
            "position_tolerance",
            positive(self.position_tolerance, "position_tolerance"),
        )
        object.__setattr__(
            self,
            "orientation_tolerance",
            positive(self.orientation_tolerance, "orientation_tolerance"),
        )
