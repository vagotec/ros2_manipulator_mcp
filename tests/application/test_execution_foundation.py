"""Focused Phase 15 execution-state and ownership tests."""

import pytest

from ros2_manipulator_mcp.application import (
    ExecutionRegistry,
    ManipulatorService,
    PlanRegistry,
    calculate_execution_timeout,
)
from ros2_manipulator_mcp.config.settings import load_settings
from ros2_manipulator_mcp.domain import (
    DomainErrorCode,
    ExecutionState,
    JointGoal,
    PlanningRequest,
    Trajectory,
    TrajectoryPoint,
)


def _request() -> PlanningRequest:
    return PlanningRequest(JointGoal("arm", ("joint1",), (0.1,)))


def _trajectory() -> Trajectory:
    return Trajectory(("joint1",), (TrajectoryPoint((0.1,), 0.0),))


def _store(registry: PlanRegistry) -> None:
    registry.store(
        request=_request(),
        trajectory=_trajectory(),
        planning_duration_seconds=0.1,
        scene_revision="scene-1",
        policy_id="default-v1",
    )


def _service(
    plans: PlanRegistry,
    executions: ExecutionRegistry,
    *,
    enabled: bool,
) -> ManipulatorService:
    unused_backend = object()
    return ManipulatorService(
        description=unused_backend,
        state=unused_backend,
        kinematics=unused_backend,
        planning=unused_backend,
        scene=unused_backend,
        plans=plans,
        executions=executions,
        execution_enabled=enabled,
    )


def test_execution_state_machine_accepts_flow_and_rejects_terminal_transition() -> None:
    """Public state cannot skip required phases or leave a terminal state."""
    registry = ExecutionRegistry(id_factory=lambda: "execution")
    execution = registry.create("plan").value

    for state in (
        ExecutionState.VALIDATING,
        ExecutionState.STARTING,
        ExecutionState.RUNNING,
        ExecutionState.CANCELLING,
        ExecutionState.CANCELLED,
    ):
        execution = registry.transition(execution.execution_id, state).value

    rejected = registry.transition(execution.execution_id, ExecutionState.RUNNING)

    assert execution.state is ExecutionState.CANCELLED
    assert execution.cancellation_requested is True
    assert rejected.error.code is DomainErrorCode.INVALID_EXECUTION_TRANSITION


def test_disabled_preparation_leaves_plan_unreserved() -> None:
    """The default safety gate fails before creating execution ownership."""
    plans = PlanRegistry(id_factory=lambda: "plan")
    _store(plans)
    executions = ExecutionRegistry(id_factory=lambda: "execution")
    service = _service(plans, executions, enabled=False)

    result = service.prepare_execution("plan")
    reservation = plans.reserve_for_execution("plan", "later-execution")

    assert result.error.code is DomainErrorCode.EXECUTION_DISABLED
    assert reservation.succeeded
    assert executions.has_active_execution() is False


def test_reserved_plan_survives_expiry_and_cannot_be_evicted_or_discarded() -> None:
    """Active preparation pins its trusted trajectory and provenance."""
    now = [0.0]
    plans = PlanRegistry(
        max_plans=1,
        ttl_seconds=1.0,
        clock=lambda: now[0],
        id_factory=lambda: "plan",
    )
    _store(plans)
    plans.reserve_for_execution("plan", "execution")
    now[0] = 10.0

    discarded = plans.discard("plan")
    with pytest.raises(RuntimeError, match="capacity is reserved"):
        plans.store(
            request=_request(),
            trajectory=_trajectory(),
            planning_duration_seconds=0.1,
            scene_revision="scene-2",
        )

    assert discarded.error.code is DomainErrorCode.PLAN_RESERVED
    assert plans.get("plan").succeeded


def test_release_restores_availability_and_consumption_prevents_reuse() -> None:
    """Pre-acceptance release is reversible; accepted-goal consumption is not."""
    plans = PlanRegistry(id_factory=lambda: "plan")
    _store(plans)

    assert plans.reserve_for_execution("plan", "first").succeeded
    assert plans.release_reservation("plan", "first").succeeded
    assert plans.reserve_for_execution("plan", "second").succeeded
    assert plans.consume_reservation("plan", "second").succeeded

    reused = plans.reserve_for_execution("plan", "third")
    assert reused.error.code is DomainErrorCode.PLAN_CONSUMED


def test_second_execution_is_rejected_while_preparation_is_active() -> None:
    """The application owns one active execution and never queues another."""
    ids = iter(("plan-1", "plan-2"))
    plans = PlanRegistry(id_factory=lambda: next(ids))
    _store(plans)
    _store(plans)
    service = _service(
        plans,
        ExecutionRegistry(id_factory=lambda: "execution"),
        enabled=True,
    )

    first = service.prepare_execution("plan-1")
    second = service.prepare_execution("plan-2")

    assert first.value.state is ExecutionState.VALIDATING
    assert second.error.code is DomainErrorCode.EXECUTION_CONFLICT
    assert plans.get("plan-2").succeeded


def test_timeout_calculation_uses_policy_formula_and_hard_maximum() -> None:
    """Future outer waits use explicit policy without silent invalid values."""
    settings = load_settings().execution

    ordinary = calculate_execution_timeout(
        3.0,
        multiplier=settings.timeout_multiplier,
        margin_seconds=settings.timeout_margin_sec,
        maximum_seconds=settings.timeout_max_sec,
    )
    bounded = calculate_execution_timeout(
        100.0,
        multiplier=settings.timeout_multiplier,
        margin_seconds=settings.timeout_margin_sec,
        maximum_seconds=settings.timeout_max_sec,
    )

    assert settings.enabled is False
    assert ordinary == 8.0
    assert bounded == 30.0
    with pytest.raises(ValueError):
        calculate_execution_timeout(
            -1.0,
            multiplier=settings.timeout_multiplier,
            margin_seconds=settings.timeout_margin_sec,
            maximum_seconds=settings.timeout_max_sec,
        )
