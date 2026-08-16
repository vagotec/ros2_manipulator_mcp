"""Backend-neutral execution state and status."""

from dataclasses import dataclass, replace
from enum import StrEnum
from math import isfinite

from ros2_manipulator_mcp.domain._validation import identifier
from ros2_manipulator_mcp.domain.results import DomainFailure


class ExecutionState(StrEnum):
    """Stable states visible to future execution clients."""

    PENDING = "pending"
    VALIDATING = "validating"
    STARTING = "starting"
    RUNNING = "running"
    CANCELLING = "cancelling"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"
    TIMED_OUT = "timed_out"

    @property
    def terminal(self) -> bool:
        """Return whether no further transition is legal."""
        return self in _TERMINAL_STATES


_TERMINAL_STATES = frozenset(
    {
        ExecutionState.SUCCEEDED,
        ExecutionState.FAILED,
        ExecutionState.CANCELLED,
        ExecutionState.TIMED_OUT,
    }
)

_LEGAL_TRANSITIONS: dict[ExecutionState, frozenset[ExecutionState]] = {
    ExecutionState.PENDING: frozenset(
        {ExecutionState.VALIDATING, ExecutionState.FAILED}
    ),
    ExecutionState.VALIDATING: frozenset(
        {ExecutionState.STARTING, ExecutionState.FAILED}
    ),
    ExecutionState.STARTING: frozenset(
        {
            ExecutionState.RUNNING,
            ExecutionState.CANCELLING,
            ExecutionState.FAILED,
            ExecutionState.TIMED_OUT,
        }
    ),
    ExecutionState.RUNNING: frozenset(
        {
            ExecutionState.SUCCEEDED,
            ExecutionState.FAILED,
            ExecutionState.CANCELLING,
            ExecutionState.TIMED_OUT,
        }
    ),
    ExecutionState.CANCELLING: frozenset(
        {
            ExecutionState.CANCELLED,
            ExecutionState.FAILED,
            ExecutionState.TIMED_OUT,
        }
    ),
}


@dataclass(frozen=True)
class ExecutionRecord:
    """Immutable current status of one application-owned execution."""

    execution_id: str
    plan_id: str
    state: ExecutionState = ExecutionState.PENDING
    failure: DomainFailure | None = None
    cancellation_requested: bool = False
    backend_goal_accepted: bool = False
    backend_stop_requested: bool = False
    backend_stop_acknowledged: bool = False
    execution_terminal: bool = False
    state_stabilized: bool = False
    physical_stop_confirmed: bool = False

    def __post_init__(self) -> None:
        identifier(self.execution_id, "execution_id")
        identifier(self.plan_id, "plan_id")
        if not isinstance(self.state, ExecutionState):
            raise ValueError("state must be an ExecutionState")
        if self.failure is not None and self.state not in {
            ExecutionState.FAILED,
            ExecutionState.TIMED_OUT,
        }:
            raise ValueError("failure is allowed only for failed or timed-out execution")
        if self.physical_stop_confirmed:
            raise ValueError("physical stop confirmation is not supported")

    def transition(
        self,
        state: ExecutionState,
        *,
        failure: DomainFailure | None = None,
    ) -> "ExecutionRecord":
        """Return the next immutable record or reject an illegal transition."""
        if not isinstance(state, ExecutionState):
            raise ValueError("state must be an ExecutionState")
        if state not in _LEGAL_TRANSITIONS.get(self.state, frozenset()):
            raise ValueError(
                f"execution cannot transition from {self.state.value} to {state.value}"
            )
        return replace(
            self,
            state=state,
            failure=failure,
            cancellation_requested=(
                self.cancellation_requested or state is ExecutionState.CANCELLING
            ),
        )

    def with_cancellation_status(
        self,
        *,
        backend_stop_requested: bool | None = None,
        backend_goal_accepted: bool | None = None,
        backend_stop_acknowledged: bool | None = None,
        execution_terminal: bool | None = None,
        state_stabilized: bool | None = None,
    ) -> "ExecutionRecord":
        """Return updated cancellation evidence without changing state."""
        return replace(
            self,
            backend_stop_requested=(
                self.backend_stop_requested
                if backend_stop_requested is None
                else backend_stop_requested
            ),
            backend_goal_accepted=(
                self.backend_goal_accepted
                if backend_goal_accepted is None
                else backend_goal_accepted
            ),
            backend_stop_acknowledged=(
                self.backend_stop_acknowledged
                if backend_stop_acknowledged is None
                else backend_stop_acknowledged
            ),
            execution_terminal=(
                self.execution_terminal
                if execution_terminal is None
                else execution_terminal
            ),
            state_stabilized=(
                self.state_stabilized
                if state_stabilized is None
                else state_stabilized
            ),
        )


@dataclass(frozen=True)
class ExecutionValidationFinding:
    """One stable reason why execution validation was rejected."""

    code: str
    message: str
    joint_name: str | None = None
    expected_position: float | None = None
    measured_position: float | None = None
    absolute_deviation: float | None = None
    tolerance: float | None = None

    def __post_init__(self) -> None:
        identifier(self.code, "finding code")
        identifier(self.message, "finding message")
        if self.joint_name is not None:
            identifier(self.joint_name, "joint_name")
        numeric = (
            self.expected_position,
            self.measured_position,
            self.absolute_deviation,
            self.tolerance,
        )
        if any(
            value is not None
            and (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not isfinite(value)
            )
            for value in numeric
        ):
            raise ValueError("finding numeric details must be finite numbers")
        if self.absolute_deviation is not None and self.absolute_deviation < 0:
            raise ValueError("absolute_deviation must be non-negative")
        if self.tolerance is not None and self.tolerance < 0:
            raise ValueError("tolerance must be non-negative")


@dataclass(frozen=True)
class ExecutionValidationResult:
    """Bounded result of application pre-execution validation."""

    execution_id: str
    plan_id: str
    policy_id: str
    findings: tuple[ExecutionValidationFinding, ...] = ()
    state_age_seconds: float | None = None
    start_state_matches: bool | None = None
    scene_revision_matches: bool | None = None

    def __post_init__(self) -> None:
        identifier(self.execution_id, "execution_id")
        identifier(self.plan_id, "plan_id")
        identifier(self.policy_id, "policy_id")
        object.__setattr__(self, "findings", tuple(self.findings))
        if self.state_age_seconds is not None and (
            not isfinite(self.state_age_seconds) or self.state_age_seconds < 0
        ):
            raise ValueError("state_age_seconds must be finite and non-negative")

    @property
    def valid(self) -> bool:
        """Return whether every required validation check passed."""
        return not self.findings
