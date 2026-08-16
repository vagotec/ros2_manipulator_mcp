"""Backend-neutral execution boundary."""

from dataclasses import dataclass
from enum import StrEnum
from typing import Protocol

from ros2_manipulator_mcp.domain import DomainResult, Trajectory


class BackendExecutionOutcome(StrEnum):
    """Stable terminal outcomes returned by an execution backend."""

    SUCCEEDED = "succeeded"
    PREEMPTED = "preempted"
    FAILED = "failed"


@dataclass(frozen=True)
class BackendExecutionResult:
    """Backend-neutral terminal result for one accepted execution."""

    outcome: BackendExecutionOutcome
    detail: str = ""


class ExecutionPort(Protocol):
    """Submit and stop application-owned trajectories without ROS leakage."""

    async def submit_execution(
        self,
        execution_id: str,
        trajectory: Trajectory,
        *,
        server_timeout_seconds: float,
        acceptance_timeout_seconds: float,
    ) -> DomainResult[bool]: ...

    async def wait_execution_result(
        self,
        execution_id: str,
        *,
        timeout_seconds: float,
    ) -> DomainResult[BackendExecutionResult]: ...

    async def request_execution_stop(
        self,
        execution_id: str,
    ) -> DomainResult[bool]: ...

    def quarantine_execution_backend(self, execution_id: str) -> None: ...

    def close(self) -> None: ...
