"""Real MCP 2026-07-28 client/server tests over stdio."""

import asyncio
import json
from pathlib import Path

from mcp.client.session import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client


PROJECT_ROOT = Path(__file__).parents[2]
SERVER_SCRIPT = PROJECT_ROOT / "tests" / "mcp" / "stdio_test_server.py"
PYTHON = PROJECT_ROOT / ".venv" / "bin" / "python"


def _parameters() -> StdioServerParameters:
    environment = {
        "PYTHONPATH": str(PROJECT_ROOT / "src"),
        "ROS2_MANIPULATOR_MCP_CONFIG": str(
            PROJECT_ROOT / "src" / "ros2_manipulator_mcp" / "config" / "default.toml"
        ),
        "UV_CACHE_DIR": "/tmp/ros2_manipulator_mcp_uv_cache",
    }
    return StdioServerParameters(
        command=str(PYTHON),
        args=[str(SERVER_SCRIPT)],
        env=environment,
        cwd=PROJECT_ROOT,
    )


async def _with_session(operation):
    async with stdio_client(_parameters()) as (read_stream, write_stream):
        async with ClientSession(read_stream, write_stream) as session:
            discovery = await session.discover()
            return await operation(session, discovery)


def test_stdio_discovery_and_surface_counts() -> None:
    async def exercise(session, discovery):
        tools = await session.list_tools()
        resources = await session.list_resources()
        templates = await session.list_resource_templates()
        prompts = await session.list_prompts()
        return discovery, tools, resources, templates, prompts

    discovery, tools, resources, templates, prompts = asyncio.run(
        _with_session(exercise)
    )

    assert "2026-07-28" in discovery.supported_versions
    assert discovery.meta["io.modelcontextprotocol/serverInfo"] == {
        "name": "ros2-manipulator-mcp", "version": "0.1.0"
    }
    assert discovery.capabilities.tools is not None
    assert discovery.capabilities.resources is not None
    assert discovery.capabilities.prompts is not None
    assert "Physical execution" in discovery.instructions
    assert (len(tools.tools), len(resources.resources), len(templates.resource_templates), len(prompts.prompts)) == (
        19, 7, 3, 6
    )


def test_stdio_tool_results_preserve_structured_error_contract() -> None:
    async def exercise(session, discovery):
        description = await session.call_tool("describe_manipulator")
        rejected = await session.call_tool("apply_collision_object", {
            "collision_object": {
                "object_id": "oversized", "primitive_type": "box",
                "dimensions": [0.2, 0.2, 0.2],
                "pose": {
                    "frame_id": "base",
                    "position": {"x": 0, "y": 0, "z": 0},
                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                },
            }
        })
        unexpected = await session.call_tool(
            "get_end_effector_pose",
            {"planning_group": "test-group", "target_link": "tool"},
        )
        return description, rejected, unexpected

    description, rejected, unexpected = asyncio.run(_with_session(exercise))

    assert description.structured_content["data"]["manipulator_id"] == "stdio-test-arm"
    assert description.is_error is not True
    assert rejected.is_error is True
    assert rejected.structured_content["error_code"] == "policy_rejected"
    assert unexpected.is_error is True
    assert unexpected.structured_content["error_code"] == "backend_error"
    assert "private backend detail" not in json.dumps(unexpected.structured_content)


def test_stdio_resource_and_prompt_protocol_paths() -> None:
    async def exercise(session, discovery):
        resource = await session.read_resource("manipulator://overview")
        prompt = await session.get_prompt(
            "safe_manipulation_workflow",
            {"goal_summary": "Create a reviewable test plan"},
        )
        return resource, prompt

    resource, prompt = asyncio.run(_with_session(exercise))
    overview = json.loads(resource.contents[0].text)
    prompt_text = prompt.messages[0].content.text

    assert overview["data"]["manipulator_id"] == "stdio-test-arm"
    assert resource.contents[0].mime_type == "application/json"
    assert "STOP" in prompt_text
    assert "Do not execute motion" in prompt_text
