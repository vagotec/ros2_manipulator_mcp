"""Focused tests for advisory Phase 9 MCP Prompts."""

import asyncio

from ros2_manipulator_mcp.server import create_server


def test_prompt_registration_and_argument_contracts() -> None:
    server = create_server()
    prompts = {prompt.name: prompt for prompt in asyncio.run(server.list_prompts())}

    assert set(prompts) == {
        "inspect_manipulator",
        "diagnose_kinematics_failure",
        "diagnose_planning_failure",
        "review_motion_plan",
        "review_planning_scene_change",
        "safe_manipulation_workflow",
        "diagnose_execution_failure",
    }
    review_arguments = {value.name: value for value in prompts["review_motion_plan"].arguments}
    workflow_arguments = {
        value.name: value for value in prompts["safe_manipulation_workflow"].arguments
    }
    assert review_arguments["plan_id"].required is True
    assert workflow_arguments["goal_summary"].required is True
    assert workflow_arguments["planning_group"].required is False


def test_safe_workflow_stops_before_execution_and_names_existing_context() -> None:
    server = create_server()
    result = asyncio.run(server.get_prompt(
        "safe_manipulation_workflow",
        {"goal_summary": "Reach a reviewed pose goal"},
    ))
    text = result.messages[0].content.text

    assert result.messages[0].role == "user"
    assert "manipulator://safety" in text
    assert "validate_motion_plan" in text
    assert "STOP" in text
    assert "Do not execute motion" in text
    assert "not certified physical safety" in text


def test_prompt_generation_requires_no_application_or_ros_runtime() -> None:
    server = create_server()
    result = asyncio.run(server.get_prompt(
        "diagnose_kinematics_failure",
        {"failure": "No IK solution", "planning_group": "selected-group"},
    ))
    text = result.messages[0].content.text

    assert "selected-group" in text
    assert "manipulator://health" in text
    assert "do not plan" in text.lower()
