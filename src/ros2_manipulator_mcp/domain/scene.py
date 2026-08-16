"""Primitive planning-scene objects and snapshots."""

from dataclasses import dataclass
from enum import StrEnum

from ros2_manipulator_mcp.domain._validation import (
    finite_values,
    identifier,
)
from ros2_manipulator_mcp.domain.geometry import Pose
from ros2_manipulator_mcp.domain.state import RobotState


class PrimitiveType(StrEnum):
    """Primitive geometry supported by public v0.1.0 scene operations."""

    BOX = "box"
    SPHERE = "sphere"
    CYLINDER = "cylinder"
    CONE = "cone"


_DIMENSION_COUNTS = {
    PrimitiveType.BOX: 3,
    PrimitiveType.SPHERE: 1,
    PrimitiveType.CYLINDER: 2,
    PrimitiveType.CONE: 2,
}


@dataclass(frozen=True)
class CollisionPrimitive:
    """Positive dimensions for one supported primitive shape."""

    primitive_type: PrimitiveType
    dimensions: tuple[float, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.primitive_type, PrimitiveType):
            raise ValueError("primitive_type must be a supported PrimitiveType")
        dimensions = finite_values(self.dimensions, "dimensions")
        expected = _DIMENSION_COUNTS[self.primitive_type]
        if len(dimensions) != expected:
            raise ValueError(
                f"{self.primitive_type.value} requires {expected} dimensions"
            )
        if any(value <= 0 for value in dimensions):
            raise ValueError("primitive dimensions must be greater than zero")
        object.__setattr__(self, "dimensions", dimensions)


@dataclass(frozen=True)
class CollisionObject:
    """One named primitive located in the planning scene."""

    object_id: str
    primitive: CollisionPrimitive
    pose: Pose

    def __post_init__(self) -> None:
        identifier(self.object_id, "object_id")


@dataclass(frozen=True)
class PlanningSceneSnapshot:
    """A revisioned immutable view of world collision objects."""

    revision: str
    collision_objects: tuple[CollisionObject, ...]
    robot_state: RobotState | None = None

    def __post_init__(self) -> None:
        identifier(self.revision, "revision")
        objects = tuple(self.collision_objects)
        object_ids = tuple(item.object_id for item in objects)
        if len(set(object_ids)) != len(object_ids):
            raise ValueError("collision object identifiers must be unique")
        object.__setattr__(self, "collision_objects", objects)
