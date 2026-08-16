"""Manipulator motion-planning backend capability."""

from typing import Protocol

from ros2_manipulator_mcp.domain import (
    CartesianPathRequest,
    CartesianPathResult,
    DomainResult,
    PlanningRequest,
    Trajectory,
)


class MotionPlanningPort(Protocol):
    """Compute trajectories without owning public plan identities."""

    async def plan_motion(
        self,
        request: PlanningRequest,
    ) -> DomainResult[Trajectory]:
        """Compute a trajectory for a joint or pose planning request."""
        ...

    async def compute_cartesian_path(
        self,
        request: CartesianPathRequest,
    ) -> DomainResult[CartesianPathResult]:
        """Compute a Cartesian path and explicit completion fraction."""
        ...
