"""Opt-in MCP stdio E2E tests against the official mock-hardware graph."""

import asyncio
import json
import os
from pathlib import Path
from typing import Any

import pytest
from mcp.client.session import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client


PROJECT_ROOT = Path(__file__).parents[2]
SERVER = PROJECT_ROOT / ".venv" / "bin" / "ros2-manipulator-mcp"
OBJECT_ID = "phase11-mcp-box"

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(
        os.environ.get("ROS2_MANIPULATOR_MCP_RUN_LIVE_TESTS") != "1",
        reason="requires the documented OpenMANIPULATOR-X live graph",
    ),
]


def _parameters() -> StdioServerParameters:
    environment = dict(os.environ)
    python_path = os.pathsep.join(
        value
        for value in (str(PROJECT_ROOT / "src"), os.environ.get("PYTHONPATH"))
        if value
    )
    environment.update(
        {
            "PYTHONPATH": python_path,
            "ROS2_MANIPULATOR_MCP_CONFIG": str(
                PROJECT_ROOT
                / "src"
                / "ros2_manipulator_mcp"
                / "config"
                / "default.toml"
            ),
        }
    )
    return StdioServerParameters(
        command=str(SERVER),
        env=environment,
        cwd=PROJECT_ROOT,
    )


async def _with_session(operation):
    async with stdio_client(_parameters()) as (read_stream, write_stream):
        async with ClientSession(read_stream, write_stream) as session:
            discovery = await session.discover()
            return await operation(session, discovery)


def _data(result: Any) -> dict[str, Any]:
    assert result.is_error is not True, result.structured_content
    assert result.structured_content["success"] is True
    return result.structured_content["data"]


def _resource_data(result: Any) -> dict[str, Any]:
    payload = json.loads(result.contents[0].text)
    assert payload["success"] is True, payload
    return payload["data"]


def _joint_state_input(state: dict[str, Any]) -> dict[str, Any]:
    return {
        "joint_names": state["joint_names"],
        "positions": state["positions"],
        "velocities": state["velocities"],
        "efforts": state["efforts"],
        "timestamp_seconds": state["timestamp_seconds"],
    }


def test_live_mcp_discovery_read_and_kinematics_workflow() -> None:
    """The public read surface reaches the real OpenMANIPULATOR-X model."""

    async def exercise(session, discovery):
        tools = await session.list_tools()
        resources = await session.list_resources()
        templates = await session.list_resource_templates()
        prompts = await session.list_prompts()

        descriptor = _data(await session.call_tool("describe_manipulator"))
        groups = _data(await session.call_tool("list_planning_groups"))
        state = _data(await session.call_tool("get_current_robot_state"))
        pose = _data(
            await session.call_tool(
                "get_end_effector_pose",
                {"planning_group": "arm", "target_link": "end_effector_link"},
            )
        )["pose"]
        fk = _data(
            await session.call_tool(
                "compute_forward_kinematics",
                {
                    "planning_group": "arm",
                    "target_link": "end_effector_link",
                    "robot_state": _joint_state_input(state),
                },
            )
        )["pose"]
        ik = _data(
            await session.call_tool(
                "compute_inverse_kinematics",
                {
                    "planning_group": "arm",
                    "target_link": "end_effector_link",
                    "target_pose": fk,
                    "seed_state": _joint_state_input(state),
                },
            )
        )
        validity = _data(
            await session.call_tool(
                "check_robot_state_validity",
                {
                    "planning_group": "arm",
                    "robot_state": _joint_state_input(ik),
                },
            )
        )
        live_resources = {
            uri: _resource_data(await session.read_resource(uri))
            for uri in (
                "manipulator://overview",
                "manipulator://state/current",
                "manipulator://scene",
                "manipulator://health",
            )
        }
        return (
            discovery,
            tools,
            resources,
            templates,
            prompts,
            descriptor,
            groups,
            state,
            pose,
            fk,
            validity,
            live_resources,
        )

    (
        discovery,
        tools,
        resources,
        templates,
        prompts,
        descriptor,
        groups,
        state,
        pose,
        fk,
        validity,
        live_resources,
    ) = asyncio.run(_with_session(exercise))

    assert "2026-07-28" in discovery.supported_versions
    assert (len(tools.tools), len(resources.resources)) == (19, 7)
    assert (len(templates.resource_templates), len(prompts.prompts)) == (3, 6)
    assert descriptor["manipulator_id"] == "open_manipulator_x"
    assert any(group["name"] == "arm" for group in groups["planning_groups"])
    assert {"joint1", "joint2", "joint3", "joint4"} <= set(
        state["joint_names"]
    )
    assert pose["frame_id"] == fk["frame_id"] == "world"
    assert validity["valid"] is True
    assert live_resources["manipulator://overview"]["manipulator_id"] == (
        "open_manipulator_x"
    )
    assert live_resources["manipulator://state/current"]["robot_state"]
    assert "collision_objects" in live_resources["manipulator://scene"]
    assert live_resources["manipulator://health"]["ready"] is True


def test_live_mcp_planning_registry_validation_and_discard_workflow() -> None:
    """Plan-only MCP calls produce, validate, retrieve, and discard plans."""

    async def exercise(session, discovery):
        del discovery
        state = _data(await session.call_tool("get_current_robot_state"))
        start_state = _joint_state_input(state)
        joint_plan = _data(
            await session.call_tool(
                "plan_to_joint_goal",
                {
                    "goal": {
                        "planning_group": "arm",
                        "joint_names": ["joint1", "joint2", "joint3", "joint4"],
                        "positions": [0.1, -0.4, 0.4, 0.1],
                    },
                    "start_state": start_state,
                    "max_velocity_scaling_factor": 0.2,
                    "max_acceleration_scaling_factor": 0.2,
                },
            )
        )
        plan_id = joint_plan["plan_id"]
        retrieved = _data(
            await session.call_tool("get_motion_plan", {"plan_id": plan_id})
        )
        validation = _data(
            await session.call_tool("validate_motion_plan", {"plan_id": plan_id})
        )
        plan_resource = _resource_data(
            await session.read_resource(f"manipulator://plans/{plan_id}")
        )

        current_pose = _data(
            await session.call_tool(
                "get_end_effector_pose",
                {"planning_group": "arm", "target_link": "end_effector_link"},
            )
        )["pose"]
        target_pose = json.loads(json.dumps(current_pose))
        target_pose["position"]["x"] -= 0.01
        pose_plan = _data(
            await session.call_tool(
                "plan_to_pose_goal",
                {
                    "goal": {
                        "planning_group": "arm",
                        "target_link": "end_effector_link",
                        "pose": target_pose,
                    },
                    "start_state": start_state,
                    "max_velocity_scaling_factor": 0.2,
                    "max_acceleration_scaling_factor": 0.2,
                },
            )
        )
        pose_plan_id = pose_plan["plan_id"]
        try:
            discarded = _data(
                await session.call_tool("discard_motion_plan", {"plan_id": plan_id})
            )
            missing = await session.call_tool(
                "get_motion_plan", {"plan_id": plan_id}
            )
        finally:
            await session.call_tool(
                "discard_motion_plan", {"plan_id": pose_plan_id}
            )
        return joint_plan, retrieved, validation, plan_resource, pose_plan, discarded, missing

    joint_plan, retrieved, validation, plan_resource, pose_plan, discarded, missing = (
        asyncio.run(_with_session(exercise))
    )

    assert joint_plan["trajectory"]["joint_names"] == [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
    ]
    assert joint_plan["trajectory"]["point_count"] > 0
    assert joint_plan["trajectory"]["duration_seconds"] > 0
    assert retrieved["plan_id"] == joint_plan["plan_id"]
    assert validation["valid"] is True
    assert plan_resource["motion_plan"]["plan_id"] == joint_plan["plan_id"]
    assert plan_resource["validation"]["valid"] is True
    assert pose_plan["trajectory"]["point_count"] > 0
    assert discarded["discarded"] is True
    assert missing.is_error is True
    assert missing.structured_content["error_code"] == "not_found"


def test_live_mcp_scene_round_trip_and_policy_rejection_workflow() -> None:
    """Scene mutation crosses policy and MoveIt; rejected input never appears."""

    async def exercise(session, discovery):
        del discovery
        initial = _data(await session.call_tool("get_planning_scene"))
        if any(item["object_id"] == OBJECT_ID for item in initial["collision_objects"]):
            _data(
                await session.call_tool(
                    "remove_collision_object", {"object_id": OBJECT_ID}
                )
            )

        collision_object = {
            "object_id": OBJECT_ID,
            "primitive_type": "box",
            "dimensions": [0.02, 0.02, 0.02],
            "pose": {
                "frame_id": "world",
                "position": {"x": 0.5, "y": 0.0, "z": 0.5},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }
        try:
            added = _data(
                await session.call_tool(
                    "apply_collision_object",
                    {"collision_object": collision_object},
                )
            )
            observed = _data(
                await session.call_tool(
                    "get_collision_object", {"object_id": OBJECT_ID}
                )
            )
            scene_resource = _resource_data(
                await session.read_resource("manipulator://scene")
            )
            rejected_id = "phase11-policy-rejected-box"
            rejected = await session.call_tool(
                "apply_collision_object",
                {
                    "collision_object": {
                        **collision_object,
                        "object_id": rejected_id,
                        "dimensions": [5.01, 0.02, 0.02],
                    }
                },
            )
            rejected_lookup = await session.call_tool(
                "get_collision_object", {"object_id": rejected_id}
            )
        finally:
            removed = _data(
                await session.call_tool(
                    "remove_collision_object", {"object_id": OBJECT_ID}
                )
            )
            final_scene = _data(await session.call_tool("get_planning_scene"))
        return (
            added,
            observed,
            scene_resource,
            rejected,
            rejected_lookup,
            removed,
            final_scene,
        )

    (
        added,
        observed,
        scene_resource,
        rejected,
        rejected_lookup,
        removed,
        final_scene,
    ) = asyncio.run(_with_session(exercise))

    assert any(item["object_id"] == OBJECT_ID for item in added["collision_objects"])
    assert observed["collision_object"]["dimensions"] == [0.02, 0.02, 0.02]
    assert any(
        item["object_id"] == OBJECT_ID
        for item in scene_resource["collision_objects"]
    )
    assert rejected.is_error is True
    assert rejected.structured_content["error_code"] == "policy_rejected"
    assert rejected_lookup.is_error is True
    assert rejected_lookup.structured_content["error_code"] == "not_found"
    assert all(item["object_id"] != OBJECT_ID for item in removed["collision_objects"])
    assert all(
        item["object_id"] != OBJECT_ID for item in final_scene["collision_objects"]
    )
