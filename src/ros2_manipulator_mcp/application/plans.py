"""Bounded in-memory storage for application-owned motion plans."""

from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass
from math import isfinite
from secrets import token_urlsafe
from time import monotonic

from ros2_manipulator_mcp.domain import (
    DomainErrorCode,
    DomainFailure,
    DomainResult,
    MotionPlan,
    PlanningRequest,
    Trajectory,
)


@dataclass(frozen=True)
class _PlanEntry:
    """A plan with application-only expiry metadata."""

    plan: MotionPlan
    expires_at: float | None
    policy_id: str


class PlanRegistry:
    """Keep a bounded set of immutable plans in process memory."""

    def __init__(
        self,
        *,
        max_plans: int = 32,
        ttl_seconds: float | None = 300.0,
        clock: Callable[[], float] = monotonic,
        id_factory: Callable[[], str] = lambda: token_urlsafe(18),
    ) -> None:
        if (
            not isinstance(max_plans, int)
            or isinstance(max_plans, bool)
            or max_plans < 1
        ):
            raise ValueError("max_plans must be a positive integer")
        if ttl_seconds is not None and (
            not isfinite(ttl_seconds) or ttl_seconds <= 0
        ):
            raise ValueError("ttl_seconds must be greater than zero")
        self._max_plans = max_plans
        self._ttl_seconds = ttl_seconds
        self._clock = clock
        self._id_factory = id_factory
        self._entries: OrderedDict[str, _PlanEntry] = OrderedDict()

    def store(
        self,
        *,
        request: PlanningRequest,
        trajectory: Trajectory,
        planning_duration_seconds: float,
        scene_revision: str,
        policy_id: str = "unversioned",
    ) -> MotionPlan:
        """Create and store a plan under a fresh opaque identifier."""
        now = self._clock()
        self._remove_expired(now)
        plan_id = self._new_plan_id()
        plan = MotionPlan(
            plan_id=plan_id,
            request=request,
            trajectory=trajectory,
            planning_duration_seconds=planning_duration_seconds,
            scene_revision=scene_revision,
        )
        expires_at = (
            None if self._ttl_seconds is None else now + self._ttl_seconds
        )
        self._entries[plan_id] = _PlanEntry(plan, expires_at, policy_id)
        while len(self._entries) > self._max_plans:
            self._entries.popitem(last=False)
        return plan

    def get(self, plan_id: str) -> DomainResult[MotionPlan]:
        """Retrieve an unexpired plan or return a stable not-found result."""
        self._remove_expired(self._clock())
        entry = self._entries.get(plan_id)
        if entry is None:
            return self._not_found(plan_id)
        return DomainResult(value=entry.plan)

    def discard(self, plan_id: str) -> DomainResult[bool]:
        """Discard an unexpired plan by identifier."""
        self._remove_expired(self._clock())
        if self._entries.pop(plan_id, None) is None:
            return self._not_found(plan_id)
        return DomainResult(value=True)

    def validation_context(
        self,
        plan_id: str,
    ) -> DomainResult[tuple[MotionPlan, str]]:
        """Return a plan and the policy identity recorded at creation."""
        self._remove_expired(self._clock())
        entry = self._entries.get(plan_id)
        if entry is None:
            return self._not_found(plan_id)
        return DomainResult(value=(entry.plan, entry.policy_id))

    def _new_plan_id(self) -> str:
        """Generate a non-empty identifier not currently in the registry."""
        for _ in range(10):
            candidate = self._id_factory()
            if candidate.strip() and candidate not in self._entries:
                return candidate
        raise RuntimeError("could not generate a unique plan identifier")

    def _remove_expired(self, now: float) -> None:
        """Remove entries whose configured lifetime has elapsed."""
        expired = tuple(
            plan_id
            for plan_id, entry in self._entries.items()
            if entry.expires_at is not None and entry.expires_at <= now
        )
        for plan_id in expired:
            del self._entries[plan_id]

    @staticmethod
    def _not_found(plan_id: str) -> DomainResult:
        return DomainResult(
            error=DomainFailure(
                DomainErrorCode.NOT_FOUND,
                f"Motion plan '{plan_id}' was not found or has expired.",
            )
        )
