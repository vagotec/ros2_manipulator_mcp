"""Stable backend-neutral result and error concepts."""

from dataclasses import dataclass
from enum import StrEnum
from typing import Generic, TypeVar

from ros2_manipulator_mcp.domain._validation import identifier


class DomainErrorCode(StrEnum):
    """Stable failure categories independent of any backend."""

    INVALID_REQUEST = "invalid_request"
    POLICY_REJECTED = "policy_rejected"
    UNAVAILABLE = "unavailable"
    TIMEOUT = "timeout"
    NOT_FOUND = "not_found"
    NO_SOLUTION = "no_solution"
    IN_COLLISION = "in_collision"
    PLANNING_FAILED = "planning_failed"
    STALE_STATE = "stale_state"
    BACKEND_ERROR = "backend_error"
    EXECUTION_DISABLED = "execution_disabled"
    EXECUTION_CONFLICT = "execution_conflict"
    PLAN_RESERVED = "plan_reserved"
    PLAN_CONSUMED = "plan_consumed"
    INVALID_EXECUTION_TRANSITION = "invalid_execution_transition"
    EXECUTION_VALIDATION_FAILED = "execution_validation_failed"
    EXECUTION_FAILED = "execution_failed"
    BACKEND_QUARANTINED = "backend_quarantined"


@dataclass(frozen=True)
class DomainFailure:
    """A stable error category with a human-readable explanation."""

    code: DomainErrorCode
    message: str

    def __post_init__(self) -> None:
        identifier(self.message, "message")


T = TypeVar("T")


@dataclass(frozen=True)
class DomainResult(Generic[T]):
    """A successful value or one domain failure, never both."""

    value: T | None = None
    error: DomainFailure | None = None

    def __post_init__(self) -> None:
        if (self.value is None) == (self.error is None):
            raise ValueError("a result must contain exactly one value or error")

    @property
    def succeeded(self) -> bool:
        """Return whether the operation produced a value."""
        return self.error is None
