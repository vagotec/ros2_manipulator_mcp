"""High-value invariant checks for backend-neutral domain models."""

from dataclasses import FrozenInstanceError
from math import inf, nan

import pytest

from ros2_manipulator_mcp.domain.geometry import Quaternion
from ros2_manipulator_mcp.domain.goals import JointGoal
from ros2_manipulator_mcp.domain.planning import PlanningRequest
from ros2_manipulator_mcp.domain.scene import CollisionPrimitive, PrimitiveType
from ros2_manipulator_mcp.domain.state import JointState
from ros2_manipulator_mcp.domain.trajectory import Trajectory, TrajectoryPoint


def test_joint_state_rejects_ambiguous_or_invalid_joint_data() -> None:
    """Bad names, vector sizes, or samples must not enter robot state."""
    invalid_samples = (
        (("joint1", "joint1"), (0.0, 1.0)),
        (("joint1", "joint2"), (0.0,)),
        (("joint1",), (nan,)),
    )

    for names, positions in invalid_samples:
        with pytest.raises(ValueError):
            JointState(names, positions)


def test_quaternion_must_be_finite_and_already_normalized() -> None:
    """The domain rejects orientation repair and non-finite input."""
    invalid_values = (
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, inf, 1.0),
    )

    for values in invalid_values:
        with pytest.raises(ValueError):
            Quaternion(*values)


def test_planning_request_rejects_invalid_scaling_factor() -> None:
    """Invalid speed policy cannot silently become full-scale motion."""
    goal = JointGoal("arm", ("joint1",), (0.5,))

    for factor in (0.0, -0.1, 1.1, nan):
        with pytest.raises(ValueError):
            PlanningRequest(goal, max_velocity_scaling_factor=factor)


def test_trajectory_rejects_non_monotonic_time() -> None:
    """Trajectory samples must have an unambiguous forward time order."""
    points = (
        TrajectoryPoint((0.0,), 0.0),
        TrajectoryPoint((0.5,), 0.0),
    )

    with pytest.raises(ValueError, match="strictly increase"):
        Trajectory(("joint1",), points)


def test_trajectory_is_deeply_immutable() -> None:
    """A stored plan cannot change through its trajectory or point objects."""
    trajectory = Trajectory(
        ("joint1",),
        (TrajectoryPoint((0.0,), 0.0),),
    )

    with pytest.raises(FrozenInstanceError):
        trajectory.points[0].time_from_start_seconds = 1.0  # type: ignore[misc]


def test_collision_primitive_rejects_invalid_dimensions() -> None:
    """Scene geometry must have the required finite positive dimensions."""
    invalid_primitives = (
        (PrimitiveType.BOX, (1.0, 2.0)),
        (PrimitiveType.SPHERE, (0.0,)),
        (PrimitiveType.CYLINDER, (1.0, inf)),
    )

    for primitive_type, dimensions in invalid_primitives:
        with pytest.raises(ValueError):
            CollisionPrimitive(primitive_type, dimensions)
