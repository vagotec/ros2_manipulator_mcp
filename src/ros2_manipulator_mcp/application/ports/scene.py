"""Planning-scene backend capability."""

from typing import Protocol

from ros2_manipulator_mcp.domain import (
    CollisionObject,
    DomainResult,
    PlanningSceneSnapshot,
)


class PlanningScenePort(Protocol):
    """Read and mutate primitive world collision objects."""

    async def get_planning_scene(self) -> DomainResult[PlanningSceneSnapshot]:
        """Return an immutable planning-scene snapshot."""
        ...

    async def apply_collision_object(
        self,
        collision_object: CollisionObject,
        *,
        replace: bool,
    ) -> DomainResult[PlanningSceneSnapshot]:
        """Add or explicitly replace one primitive collision object."""
        ...

    async def remove_collision_object(
        self,
        object_id: str,
    ) -> DomainResult[PlanningSceneSnapshot]:
        """Remove one world collision object."""
        ...
