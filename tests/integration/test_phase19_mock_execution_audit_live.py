"""One opt-in Phase 19 robustness audit through real MCP stdio."""

import asyncio
import os
from pathlib import Path

import pytest
from mcp.client.session import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client


ROOT = Path(__file__).parents[2]
JOINTS = ["joint1", "joint2", "joint3", "joint4"]
pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(
        os.environ.get("ROS2_MANIPULATOR_MCP_RUN_PHASE19_AUDIT") != "1",
        reason="requires the isolated official OpenMANIPULATOR-X mock graph",
    ),
]


def _parameters():
    return StdioServerParameters(
        command=str(ROOT / ".venv" / "bin" / "python"),
        args=["-m", "ros2_manipulator_mcp.server"],
        env={
            **os.environ,
            "PYTHONPATH": os.pathsep.join(filter(None, (
                str(ROOT / "src"), os.environ.get("PYTHONPATH"),
            ))),
            "ROS2_MANIPULATOR_MCP_CONFIG": str(
                ROOT / "tests" / "integration" / "phase18_mock.toml"
            ),
            "ROS_DOMAIN_ID": os.environ["ROS_DOMAIN_ID"],
        },
        cwd=ROOT,
    )


async def _state(session):
    result = await session.call_tool("get_current_robot_state")
    data = result.structured_content["data"]
    return {key: data[key] for key in (
        "joint_names", "positions", "velocities", "efforts", "timestamp_seconds",
    )}


async def _plan(session, target, *, scaling=0.2):
    result = await session.call_tool("plan_to_joint_goal", {
        "goal": {
            "planning_group": "arm", "joint_names": JOINTS,
            "positions": target,
        },
        "start_state": await _state(session),
        "max_velocity_scaling_factor": scaling,
        "max_acceleration_scaling_factor": scaling,
    })
    assert result.is_error is not True
    return result.structured_content["data"]["plan_id"]


async def _poll(session, execution_id, states, timeout=20.0):
    deadline = asyncio.get_running_loop().time() + timeout
    while asyncio.get_running_loop().time() < deadline:
        result = await session.call_tool(
            "get_execution_status", {"execution_id": execution_id}
        )
        data = result.structured_content["data"]
        if data["state"] in states:
            return data
        await asyncio.sleep(0.05)
    raise AssertionError(f"execution {execution_id} did not reach {states}")


async def _execute(
    session,
    plan_id,
    terminal=frozenset({"succeeded", "failed", "timed_out"}),
):
    result = await session.call_tool("execute_motion_plan", {"plan_id": plan_id})
    assert result.is_error is not True
    execution_id = result.structured_content["data"]["execution_id"]
    return execution_id, await _poll(session, execution_id, terminal)


def test_phase19_mock_execution_audit():
    async def exercise():
        async with stdio_client(_parameters()) as streams:
            async with ClientSession(*streams) as session:
                await session.discover()

                plan_ids = []
                execution_ids = []
                for target in (
                    [0.05, -0.30, 0.22, 0.03],
                    [-0.04, -0.24, 0.18, -0.02],
                    [0.02, -0.28, 0.20, 0.01],
                ):
                    plan_id = await _plan(session, target)
                    execution_id, terminal = await _execute(session, plan_id)
                    assert terminal["state"] == "succeeded"
                    measured = await _state(session)
                    positions = dict(zip(
                        measured["joint_names"], measured["positions"]
                    ))
                    assert all(
                        abs(positions[name] - expected) < 0.02
                        for name, expected in zip(JOINTS, target)
                    )
                    reused = await session.call_tool(
                        "execute_motion_plan", {"plan_id": plan_id}
                    )
                    assert reused.structured_content["error_code"] == "plan_consumed"
                    plan_ids.append(plan_id)
                    execution_ids.append(execution_id)
                assert len(set(plan_ids)) == 3
                assert len(set(execution_ids)) == 3

                long_plan = await _plan(
                    session, [-0.45, 0.10, -0.10, -0.25], scaling=0.02
                )
                queued_plan = await _plan(session, [0.15, -0.15, 0.10, 0.10])
                started = await session.call_tool(
                    "execute_motion_plan", {"plan_id": long_plan}
                )
                long_id = started.structured_content["data"]["execution_id"]
                await _poll(session, long_id, {"running"})
                conflict = await session.call_tool(
                    "execute_motion_plan", {"plan_id": queued_plan}
                )
                assert conflict.structured_content["error_code"] == "execution_conflict"
                first_cancel = await session.call_tool(
                    "cancel_execution", {"execution_id": long_id}
                )
                second_cancel = await session.call_tool(
                    "cancel_execution", {"execution_id": long_id}
                )
                cancelled = await _poll(session, long_id, {"cancelled", "failed"})
                assert first_cancel.structured_content["data"]["state"] == "cancelling"
                assert second_cancel.structured_content["data"]["state"] == "cancelling"
                assert cancelled["state"] == "cancelled"
                assert cancelled["backend_stop_requested"] is True
                after_terminal = await session.call_tool(
                    "cancel_execution", {"execution_id": long_id}
                )
                assert after_terminal.structured_content["error_code"] == (
                    "invalid_execution_transition"
                )

                stale_plan = await _plan(session, [0.20, -0.20, 0.12, 0.10])
                mover = await _plan(session, [-0.10, -0.35, 0.28, -0.08])
                _, moved = await _execute(session, mover)
                assert moved["state"] == "succeeded"
                stale_id, stale = await _execute(session, stale_plan)
                assert stale["state"] == "failed"
                assert stale["accepted"] is False
                assert stale["failure"]["code"] == "execution_validation_failed"
                retry = await session.call_tool(
                    "execute_motion_plan", {"plan_id": stale_plan}
                )
                assert retry.is_error is not True
                retry_id = retry.structured_content["data"]["execution_id"]
                assert retry_id != stale_id
                retry_terminal = await _poll(session, retry_id, {"failed"})
                assert retry_terminal["accepted"] is False

                scene_plan = await _plan(session, [0.0, -0.25, 0.20, 0.0])
                applied = await session.call_tool("apply_collision_object", {
                    "collision_object": {
                        "object_id": "phase19-audit-box",
                        "primitive_type": "box",
                        "dimensions": [0.02, 0.02, 0.02],
                        "pose": {
                            "frame_id": "world",
                            "position": {"x": 0.5, "y": 0.0, "z": 0.5},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        },
                    }
                })
                assert applied.is_error is not True
                scene_id, scene_terminal = await _execute(session, scene_plan)
                assert scene_terminal["state"] == "failed"
                assert scene_terminal["accepted"] is False
                scene_retry = await session.call_tool(
                    "execute_motion_plan", {"plan_id": scene_plan}
                )
                assert scene_retry.is_error is not True
                scene_retry_id = scene_retry.structured_content["data"]["execution_id"]
                assert scene_retry_id != scene_id
                await _poll(session, scene_retry_id, {"failed"})
                removed = await session.call_tool(
                    "remove_collision_object", {"object_id": "phase19-audit-box"}
                )
                assert removed.is_error is not True

                post_plan = await _plan(session, [0.0, -0.27, 0.20, 0.0])
                _, post = await _execute(session, post_plan)
                assert post["state"] == "succeeded"

                shutdown_plan = await _plan(
                    session, [0.40, 0.05, -0.05, 0.20], scaling=0.02
                )
                shutdown_started = await session.call_tool(
                    "execute_motion_plan", {"plan_id": shutdown_plan}
                )
                shutdown_id = shutdown_started.structured_content["data"][
                    "execution_id"
                ]
                await _poll(session, shutdown_id, {"running"})
                # Leaving stdio now exercises bounded active-adapter shutdown.

    asyncio.run(exercise())
