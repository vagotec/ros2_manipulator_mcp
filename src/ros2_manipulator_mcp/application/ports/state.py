"""Manipulator state backend capability."""

from typing import Protocol

from ros2_manipulator_mcp.domain import DomainResult, Pose, RobotState


class ManipulatorStatePort(Protocol):
    """Read current manipulator state and link poses."""

    async def get_current_state(self) -> DomainResult[RobotState]:
        """Return the current robot state."""
        ...

    async def get_end_effector_pose(
        self,
        group_name: str,
        end_effector_link: str,
    ) -> DomainResult[Pose]:
        """Return a group's current end-effector pose."""
        ...
