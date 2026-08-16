"""Opt-in Phase 17 production execution against official mock hardware."""

import asyncio
import os

import pytest

from ros2_manipulator_mcp.application import ExecutionRegistry, ManipulatorService, PlanRegistry
from ros2_manipulator_mcp.domain import DomainErrorCode, ExecutionState, JointGoal
from ros2_manipulator_mcp.profiles import OPEN_MANIPULATOR_X_DESCRIPTOR
from ros2_manipulator_mcp.ros.jazzy import JazzyManipulatorAdapter
from ros2_manipulator_mcp.safety import SafetyEvaluator, SafetyPolicy


pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(
        os.environ.get("ROS2_MANIPULATOR_MCP_RUN_EXECUTION_TESTS") != "1",
        reason="requires the documented OpenMANIPULATOR-X mock graph",
    ),
]


def test_production_normal_cancel_reuse_and_consumed_plan_boundary():
    async def exercise():
        adapter = JazzyManipulatorAdapter(OPEN_MANIPULATOR_X_DESCRIPTOR)
        execution_ids = iter(("normal", "cancel", "after-cancel", "reuse"))
        service = ManipulatorService(
            description=adapter, state=adapter, kinematics=adapter,
            planning=adapter, scene=adapter, execution=adapter,
            plans=PlanRegistry(),
            executions=ExecutionRegistry(id_factory=lambda: next(execution_ids)),
            execution_enabled=True,
            state_freshness_seconds=1.0,
            cancellation_timeout_seconds=5.0,
            stabilization_timeout_seconds=3.0,
            stabilization_quiet_window_seconds=0.5,
            safety=SafetyEvaluator(SafetyPolicy(
                policy_id="phase17-mock",
                allowed_planning_groups=("arm",),
                max_velocity_scaling_factor=0.2,
                max_acceleration_scaling_factor=0.2,
            )),
        )
        try:
            state = (await service.get_current_state()).value
            normal_plan = (await service.plan_to_joint_goal(
                JointGoal("arm", ("joint1", "joint2", "joint3", "joint4"),
                          (0.08, -0.35, 0.25, 0.05)),
                start_state=state,
                max_velocity_scaling_factor=0.2,
                max_acceleration_scaling_factor=0.2,
            )).value
            normal = await service.execute_motion_plan(normal_plan.plan_id)
            reached = (await service.get_current_state()).value

            cancel_plan = (await service.plan_to_joint_goal(
                JointGoal("arm", ("joint1", "joint2", "joint3", "joint4"),
                          (-0.4, 0.15, -0.15, -0.25)),
                start_state=reached,
                max_velocity_scaling_factor=0.02,
                max_acceleration_scaling_factor=0.02,
            )).value
            task = asyncio.create_task(service.execute_motion_plan(cancel_plan.plan_id))
            while (status := service.get_execution_status("cancel")).error is not None or status.value.state is not ExecutionState.RUNNING:
                await asyncio.sleep(0.02)
            await service.cancel_execution("cancel")
            cancelled = await task

            post_state = (await service.get_current_state()).value
            post_plan = (await service.plan_to_joint_goal(
                JointGoal("arm", ("joint1", "joint2", "joint3", "joint4"),
                          (0.0, -0.25, 0.2, 0.0)),
                start_state=post_state,
                max_velocity_scaling_factor=0.2,
                max_acceleration_scaling_factor=0.2,
            )).value
            after_cancel = await service.execute_motion_plan(post_plan.plan_id)
            reused = await service.execute_motion_plan(normal_plan.plan_id)

            assert normal.value.state is ExecutionState.SUCCEEDED
            reached_positions = dict(
                zip(reached.joints.joint_names, reached.joints.positions)
            )
            assert abs(reached_positions["joint1"] - 0.08) < 0.02
            assert cancelled.value.state is ExecutionState.CANCELLED
            assert cancelled.value.state_stabilized is True
            assert after_cancel.value.state is ExecutionState.SUCCEEDED
            assert reused.error.code is DomainErrorCode.PLAN_CONSUMED
        finally:
            adapter.close()

    asyncio.run(exercise())
