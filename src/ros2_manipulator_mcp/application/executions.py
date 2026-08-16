"""Bounded application ownership for execution state."""

from collections import OrderedDict
from collections.abc import Callable
from math import isfinite
from secrets import token_urlsafe
from threading import RLock

from ros2_manipulator_mcp.domain import (
    DomainErrorCode,
    DomainFailure,
    DomainResult,
    ExecutionRecord,
    ExecutionState,
)


class ExecutionRegistry:
    """Track one active execution and bounded recent terminal records."""

    def __init__(
        self,
        *,
        max_terminal_records: int = 32,
        id_factory: Callable[[], str] = lambda: token_urlsafe(18),
    ) -> None:
        if (
            not isinstance(max_terminal_records, int)
            or isinstance(max_terminal_records, bool)
            or max_terminal_records < 1
        ):
            raise ValueError("max_terminal_records must be a positive integer")
        self._max_terminal_records = max_terminal_records
        self._id_factory = id_factory
        self._records: OrderedDict[str, ExecutionRecord] = OrderedDict()
        self._active_execution_id: str | None = None
        self._lock = RLock()

    def create(self, plan_id: str) -> DomainResult[ExecutionRecord]:
        """Create the only active execution or return a stable conflict."""
        with self._lock:
            if self._active_execution_id is not None:
                return self._failure(
                    DomainErrorCode.EXECUTION_CONFLICT,
                    "Another execution is already active.",
                )
            execution_id = self._new_execution_id()
            try:
                record = ExecutionRecord(execution_id, plan_id)
            except ValueError:
                return self._failure(
                    DomainErrorCode.INVALID_REQUEST,
                    "plan_id must be a non-empty string.",
                )
            self._records[execution_id] = record
            self._active_execution_id = execution_id
            return DomainResult(value=record)

    def get(self, execution_id: str) -> DomainResult[ExecutionRecord]:
        """Return a current or retained terminal execution record."""
        with self._lock:
            record = self._records.get(execution_id)
            if record is None:
                return self._failure(
                    DomainErrorCode.NOT_FOUND,
                    f"Execution '{execution_id}' was not found.",
                )
            return DomainResult(value=record)

    def transition(
        self,
        execution_id: str,
        state: ExecutionState,
        *,
        failure: DomainFailure | None = None,
    ) -> DomainResult[ExecutionRecord]:
        """Apply one legal state transition and retain bounded history."""
        with self._lock:
            current = self._records.get(execution_id)
            if current is None:
                return self._failure(
                    DomainErrorCode.NOT_FOUND,
                    f"Execution '{execution_id}' was not found.",
                )
            try:
                updated = current.transition(state, failure=failure)
            except ValueError:
                return self._failure(
                    DomainErrorCode.INVALID_EXECUTION_TRANSITION,
                    "The requested execution state transition is not valid.",
                )
            self._records[execution_id] = updated
            if updated.state.terminal:
                if self._active_execution_id == execution_id:
                    self._active_execution_id = None
                self._trim_terminal_records()
            return DomainResult(value=updated)

    def has_active_execution(self) -> bool:
        """Return whether one non-terminal execution owns the service."""
        with self._lock:
            return self._active_execution_id is not None

    def update_cancellation_status(
        self,
        execution_id: str,
        **status: bool,
    ) -> DomainResult[ExecutionRecord]:
        """Record monotonic backend cancellation evidence."""
        with self._lock:
            current = self._records.get(execution_id)
            if current is None:
                return self._failure(
                    DomainErrorCode.NOT_FOUND,
                    f"Execution '{execution_id}' was not found.",
                )
            updated = current.with_cancellation_status(**status)
            self._records[execution_id] = updated
            return DomainResult(value=updated)

    def owns_active_execution(self, execution_id: str) -> bool:
        """Return whether the identifier owns the single active slot."""
        with self._lock:
            return self._active_execution_id == execution_id

    def _new_execution_id(self) -> str:
        for _ in range(10):
            candidate = self._id_factory()
            if candidate.strip() and candidate not in self._records:
                return candidate
        raise RuntimeError("could not generate a unique execution identifier")

    def _trim_terminal_records(self) -> None:
        terminal_ids = [
            execution_id
            for execution_id, record in self._records.items()
            if record.state.terminal
        ]
        for execution_id in terminal_ids[: -self._max_terminal_records]:
            del self._records[execution_id]

    @staticmethod
    def _failure(code: DomainErrorCode, message: str) -> DomainResult:
        return DomainResult(error=DomainFailure(code, message))


def calculate_execution_timeout(
    trajectory_duration_seconds: float,
    *,
    multiplier: float,
    margin_seconds: float,
    maximum_seconds: float,
) -> float:
    """Calculate the bounded future outer execution timeout."""
    values = {
        "trajectory_duration_seconds": trajectory_duration_seconds,
        "multiplier": multiplier,
        "margin_seconds": margin_seconds,
        "maximum_seconds": maximum_seconds,
    }
    converted = {name: float(value) for name, value in values.items()}
    if any(not isfinite(value) for value in converted.values()):
        raise ValueError("execution timeout values must be finite")
    if converted["trajectory_duration_seconds"] < 0:
        raise ValueError("trajectory_duration_seconds must be non-negative")
    if converted["multiplier"] <= 0:
        raise ValueError("multiplier must be greater than zero")
    if converted["margin_seconds"] < 0:
        raise ValueError("margin_seconds must be non-negative")
    if converted["maximum_seconds"] <= 0:
        raise ValueError("maximum_seconds must be greater than zero")
    calculated = (
        converted["trajectory_duration_seconds"] * converted["multiplier"]
        + converted["margin_seconds"]
    )
    return min(calculated, converted["maximum_seconds"])
