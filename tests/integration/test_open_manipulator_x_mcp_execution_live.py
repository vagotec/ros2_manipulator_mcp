"""One opt-in Phase 18 MCP stdio execution E2E over mock hardware."""

import asyncio
import json
import os
from pathlib import Path

import pytest
from mcp.client.session import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client


ROOT = Path(__file__).parents[2]
pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(
        os.environ.get("ROS2_MANIPULATOR_MCP_RUN_MCP_EXECUTION_TEST") != "1",
        reason="requires the isolated official OpenMANIPULATOR-X mock graph",
    ),
]


async def _poll(session, execution_id, states, timeout=15.0):
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


def test_mock_execution_lifecycle_through_real_mcp_stdio():
    async def exercise():
        params = StdioServerParameters(
            command=str(ROOT / ".venv" / "bin" / "python"),
            args=["-m", "ros2_manipulator_mcp.server"],
            env={
                **os.environ,
                "PYTHONPATH": os.pathsep.join(
                    filter(None, (str(ROOT / "src"), os.environ.get("PYTHONPATH")))
                ),
                "ROS2_MANIPULATOR_MCP_CONFIG": str(
                    ROOT / "tests" / "integration" / "phase18_mock.toml"
                ),
                "ROS_DOMAIN_ID": os.environ["ROS_DOMAIN_ID"],
            },
            cwd=ROOT,
        )
        async with stdio_client(params) as streams:
            async with ClientSession(*streams) as session:
                await session.discover()
                state_result = await session.call_tool("get_current_robot_state")
                state = state_result.structured_content["data"]
                start = {
                    "joint_names": state["joint_names"],
                    "positions": state["positions"],
                    "velocities": state["velocities"],
                    "efforts": state["efforts"],
                    "timestamp_seconds": state["timestamp_seconds"],
                }

                normal_plan = await session.call_tool("plan_to_joint_goal", {
                    "goal": {
                        "planning_group": "arm",
                        "joint_names": ["joint1", "joint2", "joint3", "joint4"],
                        "positions": [0.08, -0.35, 0.25, 0.05],
                    },
                    "start_state": start,
                    "max_velocity_scaling_factor": 0.2,
                    "max_acceleration_scaling_factor": 0.2,
                })
                plan_id = normal_plan.structured_content["data"]["plan_id"]
                started = await session.call_tool(
                    "execute_motion_plan", {"plan_id": plan_id}
                )
                normal_id = started.structured_content["data"]["execution_id"]
                normal = await _poll(session, normal_id, {"succeeded", "failed"})
                reused = await session.call_tool(
                    "execute_motion_plan", {"plan_id": plan_id}
                )

                current = await session.call_tool("get_current_robot_state")
                state = current.structured_content["data"]
                start = {key: state[key] for key in (
                    "joint_names", "positions", "velocities", "efforts",
                    "timestamp_seconds",
                )}
                cancel_plan = await session.call_tool("plan_to_joint_goal", {
                    "goal": {
                        "planning_group": "arm",
                        "joint_names": ["joint1", "joint2", "joint3", "joint4"],
                        "positions": [-0.4, 0.15, -0.15, -0.25],
                    },
                    "start_state": start,
                    "max_velocity_scaling_factor": 0.02,
                    "max_acceleration_scaling_factor": 0.02,
                })
                cancel_plan_id = cancel_plan.structured_content["data"]["plan_id"]
                started = await session.call_tool(
                    "execute_motion_plan", {"plan_id": cancel_plan_id}
                )
                cancel_id = started.structured_content["data"]["execution_id"]
                await _poll(session, cancel_id, {"starting", "running"})
                cancelling = await session.call_tool(
                    "cancel_execution", {"execution_id": cancel_id}
                )
                cancelled = await _poll(session, cancel_id, {"cancelled", "failed"})
                resource = await session.read_resource(
                    f"manipulator://executions/{cancel_id}"
                )

                post = await session.call_tool("get_current_robot_state")
                state = post.structured_content["data"]
                post_plan = await session.call_tool("plan_to_joint_goal", {
                    "goal": {
                        "planning_group": "arm",
                        "joint_names": ["joint1", "joint2", "joint3", "joint4"],
                        "positions": [0.0, -0.25, 0.2, 0.0],
                    },
                    "start_state": {key: state[key] for key in (
                        "joint_names", "positions", "velocities", "efforts",
                        "timestamp_seconds",
                    )},
                    "max_velocity_scaling_factor": 0.2,
                    "max_acceleration_scaling_factor": 0.2,
                })
                post_id = post_plan.structured_content["data"]["plan_id"]
                post_started = await session.call_tool(
                    "execute_motion_plan", {"plan_id": post_id}
                )
                after = await _poll(
                    session,
                    post_started.structured_content["data"]["execution_id"],
                    {"succeeded", "failed"},
                )

                resource_data = json.loads(resource.contents[0].text)["data"]["execution"]
                assert normal["state"] == "succeeded"
                assert reused.structured_content["error_code"] == "plan_consumed"
                assert cancelling.structured_content["data"]["state"] == "cancelling"
                assert cancelled["state"] == "cancelled"
                assert cancelled["backend_stop_requested"] is True
                assert cancelled["state_stabilized"] is True
                assert cancelled["physical_stop_confirmed"] is False
                assert resource_data["state"] == "cancelled"
                assert after["state"] == "succeeded"

    asyncio.run(exercise())
