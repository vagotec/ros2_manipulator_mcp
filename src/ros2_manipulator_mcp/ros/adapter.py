"""Composite backend-neutral manipulator adapter contract."""

from typing import Protocol, runtime_checkable

from ros2_manipulator_mcp.application.ports import (
    KinematicsPort,
    ManipulatorDescriptionPort,
    ManipulatorStatePort,
    MotionPlanningPort,
    PlanningScenePort,
)


@runtime_checkable
class ManipulatorAdapter(
    ManipulatorDescriptionPort,
    ManipulatorStatePort,
    KinematicsPort,
    MotionPlanningPort,
    PlanningScenePort,
    Protocol,
):
    """Complete replaceable backend contract used by the application."""

    def close(self) -> None:
        """Release backend resources deterministically."""
        ...
