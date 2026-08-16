"""Focused safety-policy integration tests with a fake backend."""

import asyncio

from ros2_manipulator_mcp.application import ManipulatorService, PlanRegistry
from ros2_manipulator_mcp.domain import (
    CollisionObject,
    CollisionPrimitive,
    DomainErrorCode,
    DomainResult,
    JointGoal,
    PlanningRequest,
    PlanningSceneSnapshot,
    Pose,
    PrimitiveType,
    Quaternion,
    Trajectory,
    TrajectoryPoint,
    Vector3,
)
from ros2_manipulator_mcp.safety import SafetyEvaluator, SafetyPolicy


def _request(group_name: str = "arm", planning_time: float = 1.0) -> PlanningRequest:
    return PlanningRequest(
        JointGoal(group_name, ("joint1",), (0.5,)),
        allowed_planning_time_seconds=planning_time,
    )


def _object(
    object_id: str = "fixture-table",
    dimensions: tuple[float, ...] = (1.0, 0.5, 0.1),
) -> CollisionObject:
    return CollisionObject(
        object_id,
        CollisionPrimitive(PrimitiveType.BOX, dimensions),
        Pose("base", Vector3(0.5, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
    )


class PolicyBackend:
    """Fake implementing only methods exercised by policy integration."""

    def __init__(self, scene: PlanningSceneSnapshot | None = None) -> None:
        self.scene = scene or PlanningSceneSnapshot("scene-1", ())
        self.planning_calls = 0
        self.apply_calls = 0

    async def get_planning_scene(self) -> DomainResult[PlanningSceneSnapshot]:
        return DomainResult(value=self.scene)

    async def plan_motion(
        self,
        request: PlanningRequest,
    ) -> DomainResult[Trajectory]:
        self.planning_calls += 1
        return DomainResult(
            value=Trajectory(
                ("joint1",),
                (TrajectoryPoint((0.5,), 0.0),),
            )
        )

    async def apply_collision_object(
        self,
        collision_object: CollisionObject,
        *,
        replace: bool,
    ) -> DomainResult[PlanningSceneSnapshot]:
        self.apply_calls += 1
        return DomainResult(value=self.scene)


def _service(
    backend: PolicyBackend,
    policy: SafetyPolicy,
    plans: PlanRegistry | None = None,
) -> ManipulatorService:
    return ManipulatorService(
        description=backend,  # type: ignore[arg-type]
        state=backend,  # type: ignore[arg-type]
        kinematics=backend,  # type: ignore[arg-type]
        planning=backend,
        scene=backend,
        plans=plans,
        safety=SafetyEvaluator(policy),
    )


def test_unsafe_planning_is_rejected_before_backend_invocation() -> None:
    """A policy error must stop planning before any backend planning call."""
    backend = PolicyBackend()
    service = _service(
        backend,
        SafetyPolicy(allowed_planning_groups=("arm",), max_planning_time_seconds=2),
    )

    result = asyncio.run(service.plan_motion(_request(planning_time=3.0)))

    assert result.error.code is DomainErrorCode.POLICY_REJECTED
    assert "PLANNING_TIME_EXCEEDED" in result.error.message
    assert backend.planning_calls == 0


def test_allowed_planning_reaches_backend_and_records_policy() -> None:
    """Accepted requests produce plans associated with the active policy."""
    backend = PolicyBackend()
    registry = PlanRegistry(id_factory=lambda: "safe-plan")
    service = _service(
        backend,
        SafetyPolicy(policy_id="site-policy-v1", allowed_planning_groups=("arm",)),
        registry,
    )

    result = asyncio.run(service.plan_motion(_request()))
    validation = service.prepare_plan_validation("safe-plan")

    assert result.value.plan_id == "safe-plan"
    assert validation.value.allowed is True
    assert validation.value.policy_id == "site-policy-v1"
    assert backend.planning_calls == 1


def test_oversized_collision_primitive_is_rejected_before_scene_write() -> None:
    """Bounded geometry policy prevents oversized backend scene mutations."""
    backend = PolicyBackend()
    service = _service(
        backend,
        SafetyPolicy(max_primitive_dimension=1.0),
    )

    result = asyncio.run(
        service.apply_collision_object(_object(dimensions=(2.0, 0.5, 0.1)))
    )

    assert result.error.code is DomainErrorCode.POLICY_REJECTED
    assert "PRIMITIVE_DIMENSION_EXCEEDED" in result.error.message
    assert backend.apply_calls == 0


def test_replace_policy_prevents_backend_scene_write() -> None:
    """Explicit replace intent still requires policy permission."""
    existing = _object()
    backend = PolicyBackend(PlanningSceneSnapshot("scene-1", (existing,)))
    service = _service(backend, SafetyPolicy(allow_object_replace=False))

    result = asyncio.run(
        service.apply_collision_object(existing, replace=True)
    )

    assert result.error.code is DomainErrorCode.POLICY_REJECTED
    assert "OBJECT_REPLACE_NOT_ALLOWED" in result.error.message
    assert backend.apply_calls == 0


def test_stored_plan_validation_rejects_policy_identity_change() -> None:
    """A plan cannot silently inherit a policy introduced after creation."""
    backend = PolicyBackend()
    registry = PlanRegistry(id_factory=lambda: "old-policy-plan")
    original = _service(
        backend,
        SafetyPolicy(policy_id="policy-v1"),
        registry,
    )
    asyncio.run(original.plan_motion(_request()))
    changed = _service(
        backend,
        SafetyPolicy(policy_id="policy-v2"),
        registry,
    )

    result = changed.prepare_plan_validation("old-policy-plan")

    assert result.error.code is DomainErrorCode.POLICY_REJECTED
    assert "PLAN_POLICY_MISMATCH" in result.error.message
