"""Focused Phase 18 MCP execution-surface tests."""

import asyncio
import json

from ros2_manipulator_mcp.domain import (
    DomainErrorCode,
    DomainFailure,
    DomainResult,
    ExecutionRecord,
    ExecutionState,
)
from ros2_manipulator_mcp.server import create_server


class ExecutionService:
    def __init__(self, *, enabled: bool = True) -> None:
        self.enabled = enabled
        self.record = ExecutionRecord("execution", "plan", ExecutionState.RUNNING)
        self.cancel_calls = 0

    async def start_motion_plan_execution(self, plan_id):
        if not self.enabled:
            return DomainResult(error=DomainFailure(
                DomainErrorCode.EXECUTION_DISABLED,
                "Execution is disabled by configuration.",
            ))
        self.record = ExecutionRecord("execution", plan_id, ExecutionState.RUNNING)
        return DomainResult(value=self.record)

    def get_execution_status(self, execution_id):
        if execution_id != self.record.execution_id:
            return DomainResult(error=DomainFailure(
                DomainErrorCode.NOT_FOUND, "Execution was not found."
            ))
        return DomainResult(value=self.record)

    async def cancel_execution(self, execution_id):
        if execution_id != self.record.execution_id:
            return self.get_execution_status(execution_id)
        if self.record.state is not ExecutionState.CANCELLING:
            self.cancel_calls += 1
            self.record = self.record.transition(ExecutionState.CANCELLING)
        return DomainResult(value=self.record)


def test_execution_disabled_is_a_structured_mcp_error():
    server = create_server(ExecutionService(enabled=False))

    result = asyncio.run(server.call_tool("execute_motion_plan", {"plan_id": "plan"}))

    assert result.is_error is True
    assert result.structured_content["error_code"] == "execution_disabled"
    assert result.structured_content["data"] is None


def test_execute_status_and_idempotent_cancel_are_bounded():
    service = ExecutionService()
    server = create_server(service)

    async def exercise():
        started = await server.call_tool("execute_motion_plan", {"plan_id": "trusted-plan"})
        status = await server.call_tool(
            "get_execution_status", {"execution_id": "execution"}
        )
        first = await server.call_tool("cancel_execution", {"execution_id": "execution"})
        second = await server.call_tool("cancel_execution", {"execution_id": "execution"})
        return started, status, first, second

    started, status, first, second = asyncio.run(exercise())
    assert started.structured_content["data"]["plan_id"] == "trusted-plan"
    assert status.structured_content["data"]["state"] == "running"
    assert first.structured_content["data"]["state"] == "cancelling"
    assert second.structured_content["data"]["physical_stop_confirmed"] is False
    assert service.cancel_calls == 1
    assert "trajectory_execution_event" not in str(second.structured_content)


def test_execution_resource_and_diagnostic_prompt_are_advisory():
    server = create_server(ExecutionService())

    async def exercise():
        resource = await server.read_resource("manipulator://executions/execution")
        prompt = await server.get_prompt(
            "diagnose_execution_failure", {"execution_id": "execution"}
        )
        return resource, prompt

    resource, prompt = asyncio.run(exercise())
    data = json.loads(list(resource)[0].content)["data"]["execution"]
    text = prompt.messages[0].content.text
    assert data["execution_id"] == "execution"
    assert data["physical_stop_confirmed"] is False
    assert "Do not invoke tools automatically" in text
    assert "not certified physical standstill" in text
