"""Manipulator kinematics backend capability."""

from typing import Protocol

from ros2_manipulator_mcp.domain import DomainResult, Pose, RobotState


class KinematicsPort(Protocol):
    """Compute kinematics and state validity using domain values."""

    async def compute_forward_kinematics(
        self,
        group_name: str,
        target_link: str,
        state: RobotState,
    ) -> DomainResult[Pose]:
        """Compute a target-link pose for a robot state."""
        ...

    async def compute_inverse_kinematics(
        self,
        group_name: str,
        target_link: str,
        target_pose: Pose,
        seed_state: RobotState | None = None,
    ) -> DomainResult[RobotState]:
        """Compute a robot state for a framed target pose."""
        ...

    async def check_state_validity(
        self,
        group_name: str,
        state: RobotState,
    ) -> DomainResult[bool]:
        """Report whether a state satisfies backend validity checks."""
        ...
