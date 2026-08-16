"""Minimal framed geometry used by manipulator requests."""

from dataclasses import dataclass
from math import isclose

from ros2_manipulator_mcp.domain._validation import finite, identifier


@dataclass(frozen=True)
class Vector3:
    """Three finite Cartesian coordinates."""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", finite(self.x, "x"))
        object.__setattr__(self, "y", finite(self.y, "y"))
        object.__setattr__(self, "z", finite(self.z, "z"))


@dataclass(frozen=True)
class Quaternion:
    """A finite quaternion that must already be normalized.

    Values are accepted when their squared norm is within 1e-6 of one. The
    domain never normalizes or clamps caller input.
    """

    x: float
    y: float
    z: float
    w: float

    def __post_init__(self) -> None:
        values = tuple(
            finite(value, name)
            for name, value in zip("xyzw", (self.x, self.y, self.z, self.w))
        )
        norm_squared = sum(value * value for value in values)
        if not isclose(norm_squared, 1.0, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("quaternion must be normalized")
        for name, value in zip("xyzw", values):
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class Pose:
    """Position and orientation expressed in a named frame."""

    frame_id: str
    position: Vector3
    orientation: Quaternion

    def __post_init__(self) -> None:
        identifier(self.frame_id, "frame_id")
