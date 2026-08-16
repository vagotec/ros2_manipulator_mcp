"""Backend-neutral Manipulator domain models."""

from ros2_manipulator_mcp.domain.description import (
    ManipulatorDescriptor,
    PlanningGroup,
)
from ros2_manipulator_mcp.domain.geometry import Pose, Quaternion, Vector3
from ros2_manipulator_mcp.domain.goals import JointGoal, PoseGoal
from ros2_manipulator_mcp.domain.planning import (
    CartesianPathRequest,
    CartesianPathResult,
    MotionPlan,
    PlanningRequest,
)
from ros2_manipulator_mcp.domain.results import (
    DomainErrorCode,
    DomainFailure,
    DomainResult,
)
from ros2_manipulator_mcp.domain.scene import (
    CollisionObject,
    CollisionPrimitive,
    PlanningSceneSnapshot,
    PrimitiveType,
)
from ros2_manipulator_mcp.domain.state import JointState, RobotState
from ros2_manipulator_mcp.domain.trajectory import Trajectory, TrajectoryPoint

__all__ = [
    "CartesianPathRequest",
    "CartesianPathResult",
    "CollisionObject",
    "CollisionPrimitive",
    "DomainErrorCode",
    "DomainFailure",
    "DomainResult",
    "JointGoal",
    "JointState",
    "ManipulatorDescriptor",
    "MotionPlan",
    "PlanningGroup",
    "PlanningRequest",
    "PlanningSceneSnapshot",
    "Pose",
    "PoseGoal",
    "PrimitiveType",
    "Quaternion",
    "RobotState",
    "Trajectory",
    "TrajectoryPoint",
    "Vector3",
]
