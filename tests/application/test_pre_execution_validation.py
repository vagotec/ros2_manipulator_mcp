"""Focused Phase 16 pre-execution safety-gate tests."""

import asyncio

from ros2_manipulator_mcp.application import ExecutionRegistry, ManipulatorService, PlanRegistry
from ros2_manipulator_mcp.domain import (
    DomainErrorCode,
    DomainFailure,
    DomainResult,
    ExecutionState,
    JointGoal,
    JointState,
    ManipulatorDescriptor,
    PlanningGroup,
    PlanningRequest,
    PlanningSceneSnapshot,
    RobotState,
    Trajectory,
    TrajectoryPoint,
)


class ValidationBackend:
    """Provide only deterministic descriptor, state, and scene reads."""

    def __init__(self) -> None:
        group = PlanningGroup(
            "arm",
            ("joint1", "joint2"),
            ("base", "tool"),
            "base",
            "tool",
        )
        self.descriptor = ManipulatorDescriptor("test-arm", "base", (group,))
        self.state = RobotState(
            JointState(("joint1", "joint2"), (0.1, 0.2)),
            timestamp_seconds=10.0,
            sample_age_seconds=0.05,
        )
        self.scene = PlanningSceneSnapshot("scene-1", ())
        self.state_failure: DomainFailure | None = None
        self.scene_mutations = 0

    async def describe_manipulator(self):
        return DomainResult(value=self.descriptor)

    async def get_current_state(self):
        if self.state_failure is not None:
            return DomainResult(error=self.state_failure)
        return DomainResult(value=self.state)

    async def get_planning_scene(self):
        return DomainResult(value=self.scene)

    async def remove_collision_object(self, object_id):
        self.scene_mutations += 1
        return DomainResult(value=self.scene)


def _fixture() -> tuple[ManipulatorService, PlanRegistry, ValidationBackend]:
    backend = ValidationBackend()
    plans = PlanRegistry(id_factory=lambda: "plan")
    plans.store(
        request=PlanningRequest(
            JointGoal("arm", ("joint1", "joint2"), (0.3, 0.4)),
            max_velocity_scaling_factor=0.5,
            max_acceleration_scaling_factor=0.5,
        ),
        trajectory=Trajectory(
            ("joint1", "joint2"),
            (TrajectoryPoint((0.1, 0.2), 0.0),),
        ),
        planning_duration_seconds=0.1,
        scene_revision="scene-1",
        policy_id="default-v1",
    )
    service = ManipulatorService(
        description=backend,
        state=backend,
        kinematics=backend,
        planning=backend,
        scene=backend,
        plans=plans,
        executions=ExecutionRegistry(id_factory=lambda: "execution"),
        execution_enabled=True,
    )
    return service, plans, backend


def _prepare_and_validate(service: ManipulatorService):
    preparation = service.prepare_execution("plan")
    return asyncio.run(service.validate_execution(preparation.value.execution_id))


def test_fresh_matching_state_and_scene_pass_and_keep_plan_reserved() -> None:
    """Only the complete successful gate reaches STARTING without consumption."""
    service, plans, _ = _fixture()

    validation = _prepare_and_validate(service)
    status = service.get_execution_status("execution")
    discard = plans.discard("plan")

    assert validation.value.valid is True
    assert validation.value.state_age_seconds == 0.05
    assert validation.value.start_state_matches is True
    assert validation.value.scene_revision_matches is True
    assert status.value.state is ExecutionState.STARTING
    assert discard.error.code is DomainErrorCode.PLAN_RESERVED


def test_stale_state_rejects_and_releases_reservation() -> None:
    """A newly read but over-age sample cannot authorize later motion."""
    service, plans, backend = _fixture()
    backend.state = RobotState(
        backend.state.joints,
        timestamp_seconds=10.0,
        sample_age_seconds=0.251,
    )

    validation = _prepare_and_validate(service)

    assert [item.code for item in validation.value.findings] == ["STATE_STALE"]
    assert service.get_execution_status("execution").value.state is ExecutionState.FAILED
    assert plans.discard("plan").value is True


def test_start_state_mismatch_returns_joint_specific_details() -> None:
    """Deviation evidence identifies the expected and measured joint values."""
    service, plans, backend = _fixture()
    backend.state = RobotState(
        JointState(("joint1", "joint2"), (0.13, 0.2)),
        timestamp_seconds=10.0,
        sample_age_seconds=0.01,
    )

    validation = _prepare_and_validate(service)
    finding = validation.value.findings[0]

    assert finding.code == "START_STATE_MISMATCH"
    assert finding.joint_name == "joint1"
    assert finding.expected_position == 0.1
    assert finding.measured_position == 0.13
    assert finding.absolute_deviation == 0.03
    assert finding.tolerance == 0.02
    assert plans.discard("plan").value is True


def test_scene_revision_mismatch_rejects_without_replanning() -> None:
    """Exact revision policy rejects any changed primitive-world snapshot."""
    service, plans, backend = _fixture()
    backend.scene = PlanningSceneSnapshot("scene-2", ())

    validation = _prepare_and_validate(service)

    assert validation.value.scene_revision_matches is False
    assert [item.code for item in validation.value.findings] == [
        "SCENE_REVISION_MISMATCH"
    ]
    assert plans.discard("plan").value is True


def test_missing_required_joint_rejects_with_stable_finding() -> None:
    """Partial current state cannot authorize a whole planning-group path."""
    service, plans, backend = _fixture()
    backend.state = RobotState(
        JointState(("joint1",), (0.1,)),
        timestamp_seconds=10.0,
        sample_age_seconds=0.01,
    )

    validation = _prepare_and_validate(service)
    finding = validation.value.findings[0]

    assert finding.code == "MEASURED_JOINT_MISSING"
    assert finding.joint_name == "joint2"
    assert validation.value.start_state_matches is False
    assert plans.discard("plan").value is True


def test_backend_state_failure_is_bounded_and_releases_plan() -> None:
    """Readiness failure becomes a stable finding rather than an exception."""
    service, plans, backend = _fixture()
    backend.state_failure = DomainFailure(
        DomainErrorCode.TIMEOUT,
        "backend-specific detail",
    )

    validation = _prepare_and_validate(service)

    assert [item.code for item in validation.value.findings] == [
        "CURRENT_STATE_UNAVAILABLE"
    ]
    assert plans.discard("plan").value is True


def test_scene_mutation_is_rejected_while_validation_is_active() -> None:
    """Preparing execution blocks world mutation before any backend write."""
    service, plans, backend = _fixture()
    service.prepare_execution("plan")

    result = asyncio.run(service.remove_collision_object("fixture"))

    assert result.error.code is DomainErrorCode.EXECUTION_CONFLICT
    assert backend.scene_mutations == 0
    assert plans.discard("plan").error.code is DomainErrorCode.PLAN_RESERVED
