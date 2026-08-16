"""Opt-in integration tests for the official 4.1.3 mock-hardware graph."""

import asyncio
import os

import pytest

from ros2_manipulator_mcp.application import ManipulatorService
from ros2_manipulator_mcp.domain import (
    CartesianPathRequest,
    CollisionObject,
    CollisionPrimitive,
    JointGoal,
    Pose,
    PoseGoal,
    PrimitiveType,
    Quaternion,
    Vector3,
)
from ros2_manipulator_mcp.profiles import OPEN_MANIPULATOR_X_DESCRIPTOR
from ros2_manipulator_mcp.ros.jazzy import JazzyManipulatorAdapter
from ros2_manipulator_mcp.safety import SafetyEvaluator, SafetyPolicy


pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(
        os.environ.get("ROS2_MANIPULATOR_MCP_RUN_LIVE_TESTS") != "1",
        reason="requires the documented OpenMANIPULATOR-X live graph",
    ),
]


def _service() -> tuple[ManipulatorService, JazzyManipulatorAdapter]:
    adapter = JazzyManipulatorAdapter(OPEN_MANIPULATOR_X_DESCRIPTOR)
    policy = SafetyPolicy(
        policy_id="open-manipulator-x-phase6",
        allowed_planning_groups=("arm",),
        max_velocity_scaling_factor=0.2,
        max_acceleration_scaling_factor=0.2,
        min_cartesian_completion_fraction=0.5,
        allowed_object_id_prefixes=("phase6-",),
        allowed_scene_frames=("world",),
    )
    service = ManipulatorService(
        description=adapter,
        state=adapter,
        kinematics=adapter,
        planning=adapter,
        scene=adapter,
        safety=SafetyEvaluator(policy),
    )
    return service, adapter


def test_live_descriptor_state_fk_ik_and_validity() -> None:
    """The profile and core kinematics interoperate with the live model."""

    async def exercise() -> None:
        service, adapter = _service()
        try:
            descriptor = await service.describe_manipulator()
            state = await service.get_current_state()
            fk = await service.compute_forward_kinematics(
                "arm", "end_effector_link", state.value
            )
            ik = await service.compute_inverse_kinematics(
                "arm", "end_effector_link", fk.value, state.value
            )
            validity = await service.check_state_validity("arm", ik.value)

            assert descriptor.value is OPEN_MANIPULATOR_X_DESCRIPTOR
            assert {"joint1", "joint2", "joint3", "joint4"} <= set(
                state.value.joints.joint_names
            )
            assert fk.value.frame_id == "world"
            assert validity.value is True
        finally:
            adapter.close()

    asyncio.run(exercise())


def test_live_safe_joint_pose_and_cartesian_planning() -> None:
    """Safety-approved planning paths produce immutable domain results."""

    async def exercise() -> None:
        service, adapter = _service()
        try:
            state = (await service.get_current_state()).value
            fk = (
                await service.compute_forward_kinematics(
                    "arm", "end_effector_link", state
                )
            ).value
            target = Pose(
                "world",
                Vector3(fk.position.x - 0.01, fk.position.y, fk.position.z),
                fk.orientation,
            )
            joint_plan = await service.plan_to_joint_goal(
                JointGoal(
                    "arm",
                    ("joint1", "joint2", "joint3", "joint4"),
                    (0.1, -0.4, 0.4, 0.1),
                ),
                start_state=state,
                max_velocity_scaling_factor=0.2,
                max_acceleration_scaling_factor=0.2,
            )
            pose_plan = await service.plan_to_pose_goal(
                PoseGoal("arm", "end_effector_link", target),
                start_state=state,
                max_velocity_scaling_factor=0.2,
                max_acceleration_scaling_factor=0.2,
            )
            cartesian = await service.compute_cartesian_path(
                CartesianPathRequest(
                    "arm",
                    "end_effector_link",
                    (fk, target),
                    0.005,
                    start_state=state,
                    max_velocity_scaling_factor=0.2,
                    max_acceleration_scaling_factor=0.2,
                )
            )

            assert joint_plan.value.trajectory.joint_names == (
                "joint1", "joint2", "joint3", "joint4"
            )
            assert pose_plan.value.trajectory.points
            assert cartesian.value.fraction >= 0.5
            assert cartesian.value.trajectory is not None
        finally:
            adapter.close()

    asyncio.run(exercise())


def test_live_safe_primitive_scene_round_trip() -> None:
    """A policy-approved primitive can be added, observed, and removed."""

    async def exercise() -> None:
        service, adapter = _service()
        object_id = "phase6-integration-box"
        collision_object = CollisionObject(
            object_id,
            CollisionPrimitive(PrimitiveType.BOX, (0.02, 0.02, 0.02)),
            Pose(
                "world",
                Vector3(0.5, 0.0, 0.5),
                Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        )
        try:
            initial = await service.get_planning_scene()
            if object_id in {
                item.object_id for item in initial.value.collision_objects
            }:
                await service.remove_collision_object(object_id)
            added = await service.apply_collision_object(collision_object)
            assert collision_object in added.value.collision_objects

            removed = await service.remove_collision_object(object_id)
            assert object_id not in {
                item.object_id for item in removed.value.collision_objects
            }
        finally:
            final_scene = await service.get_planning_scene()
            if final_scene.error is None and object_id in {
                item.object_id for item in final_scene.value.collision_objects
            }:
                await service.remove_collision_object(object_id)
            adapter.close()

    asyncio.run(exercise())
