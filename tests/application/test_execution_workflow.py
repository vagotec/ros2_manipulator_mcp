"""Focused production execution workflow tests."""

import asyncio

from ros2_manipulator_mcp.application import ExecutionRegistry, ManipulatorService, PlanRegistry
from ros2_manipulator_mcp.application.ports import BackendExecutionOutcome, BackendExecutionResult
from ros2_manipulator_mcp.domain import (
    DomainErrorCode, DomainFailure, DomainResult, ExecutionState, JointGoal,
    JointState, ManipulatorDescriptor, PlanningGroup, PlanningRequest,
    PlanningSceneSnapshot, RobotState, Trajectory, TrajectoryPoint,
)


class ExecutionBackend:
    def __init__(self, *, accepted=True, outcome=BackendExecutionOutcome.SUCCEEDED):
        group = PlanningGroup("arm", ("joint1",), ("base", "tool"), "base", "tool")
        self.descriptor = ManipulatorDescriptor("arm", "base", (group,))
        self.accepted = accepted
        self.outcome = outcome
        self.stop_calls = 0
        self.submit_calls = 0
        self.quarantined = False
        self.result_ready = asyncio.Event()
        if outcome is BackendExecutionOutcome.SUCCEEDED:
            self.result_ready.set()

    async def describe_manipulator(self):
        return DomainResult(value=self.descriptor)

    async def get_current_state(self):
        return DomainResult(value=RobotState(
            JointState(("joint1",), (0.0,), (0.0,)), sample_age_seconds=0.01
        ))

    async def get_planning_scene(self):
        return DomainResult(value=PlanningSceneSnapshot("scene", ()))

    async def submit_execution(self, execution_id, trajectory, **timeouts):
        self.submit_calls += 1
        return DomainResult(value=self.accepted)

    async def wait_execution_result(self, execution_id, *, timeout_seconds):
        try:
            await asyncio.wait_for(self.result_ready.wait(), timeout_seconds)
        except TimeoutError:
            return DomainResult(error=DomainFailure(DomainErrorCode.TIMEOUT, "timed out"))
        return DomainResult(value=BackendExecutionResult(self.outcome))

    async def request_execution_stop(self, execution_id):
        self.stop_calls += 1
        self.result_ready.set()
        return DomainResult(value=True)

    def quarantine_execution_backend(self, execution_id):
        self.quarantined = True

    def close(self):
        pass


class GatedSubmissionBackend(ExecutionBackend):
    def __init__(self):
        super().__init__(outcome=BackendExecutionOutcome.PREEMPTED)
        self.submit_gate = asyncio.Event()

    async def submit_execution(self, execution_id, trajectory, **timeouts):
        self.submit_calls += 1
        await self.submit_gate.wait()
        return DomainResult(value=True)


def fixture(backend, plan_id="plan", **service_options):
    execution_ids = iter(("execution", "execution-2", "execution-3"))
    plans = PlanRegistry(id_factory=lambda: plan_id)
    plans.store(
        request=PlanningRequest(JointGoal("arm", ("joint1",), (0.2,))),
        trajectory=Trajectory(("joint1",), (
            TrajectoryPoint((0.0,), 0.0), TrajectoryPoint((0.2,), 0.1),
        )),
        planning_duration_seconds=0.01,
        scene_revision="scene",
        policy_id="default-v1",
    )
    service = ManipulatorService(
        description=backend, state=backend, kinematics=backend, planning=backend,
        scene=backend, execution=backend, plans=plans,
        executions=ExecutionRegistry(id_factory=lambda: next(execution_ids)),
        execution_enabled=True, stabilization_timeout_seconds=0.2,
        stabilization_quiet_window_seconds=0.05,
        **service_options,
    )
    return service, plans


def test_goal_acceptance_consumes_plan_and_success_is_terminal():
    backend = ExecutionBackend()
    service, plans = fixture(backend)

    result = asyncio.run(service.execute_motion_plan("plan"))

    assert result.value.state is ExecutionState.SUCCEEDED
    assert service.prepare_execution("plan").error.code is DomainErrorCode.PLAN_CONSUMED
    assert plans.discard("plan").value is True


def test_rejection_releases_reservation_and_plan_remains_available():
    backend = ExecutionBackend(accepted=False)
    service, plans = fixture(backend)

    result = asyncio.run(service.execute_motion_plan("plan"))

    assert result.value.state is ExecutionState.FAILED
    assert plans.discard("plan").value is True


def test_cancel_dispatches_once_and_preempted_stabilized_is_cancelled():
    async def scenario():
        backend = ExecutionBackend(outcome=BackendExecutionOutcome.PREEMPTED)
        service, _ = fixture(backend)
        task = asyncio.create_task(service.execute_motion_plan("plan"))
        while (status := service.get_execution_status("execution")).error is not None or status.value.state is not ExecutionState.RUNNING:
            await asyncio.sleep(0)
        first = await service.cancel_execution("execution")
        second = await service.cancel_execution("execution")
        result = await task
        return backend, first, second, result

    backend, first, second, result = asyncio.run(scenario())
    assert first.value.state is ExecutionState.CANCELLING
    assert second.value.state is ExecutionState.CANCELLING
    assert backend.stop_calls == 1
    assert result.value.state is ExecutionState.CANCELLED
    assert result.value.state_stabilized is True
    assert result.value.physical_stop_confirmed is False


def test_success_after_stop_quarantines_and_rejects_later_execution():
    async def scenario():
        backend = ExecutionBackend(outcome=BackendExecutionOutcome.SUCCEEDED)
        backend.result_ready.clear()
        service, _ = fixture(backend)
        task = asyncio.create_task(service.execute_motion_plan("plan"))
        while (status := service.get_execution_status("execution")).error is not None or status.value.state is not ExecutionState.RUNNING:
            await asyncio.sleep(0)
        await service.cancel_execution("execution")
        result = await task
        later = await service.execute_motion_plan("plan")
        return backend, result, later

    backend, result, later = asyncio.run(scenario())
    assert result.value.state is ExecutionState.FAILED
    assert backend.quarantined is True
    assert later.error.code is DomainErrorCode.BACKEND_QUARANTINED


def test_one_active_execution_rejects_a_second_request():
    async def scenario():
        backend = ExecutionBackend(outcome=BackendExecutionOutcome.PREEMPTED)
        service, _ = fixture(backend)
        task = asyncio.create_task(service.execute_motion_plan("plan"))
        while (status := service.get_execution_status("execution")).error is not None or status.value.state is not ExecutionState.RUNNING:
            await asyncio.sleep(0)
        conflict = await service.execute_motion_plan("plan")
        await service.cancel_execution("execution")
        await task
        return conflict

    assert asyncio.run(scenario()).error.code is DomainErrorCode.EXECUTION_CONFLICT


def test_starting_cancel_latches_once_and_terminal_cancel_sends_no_stop():
    async def scenario():
        backend = GatedSubmissionBackend()
        service, _ = fixture(backend)
        task = asyncio.create_task(service.execute_motion_plan("plan"))
        while (status := service.get_execution_status("execution")).error is not None or status.value.state is not ExecutionState.STARTING:
            await asyncio.sleep(0)
        latched = await service.cancel_execution("execution")
        repeated = await service.cancel_execution("execution")
        backend.submit_gate.set()
        terminal = await task
        after = await service.cancel_execution("execution")
        return backend, latched, repeated, terminal, after

    backend, latched, repeated, terminal, after = asyncio.run(scenario())
    assert latched.value.state is ExecutionState.CANCELLING
    assert repeated.value.state is ExecutionState.CANCELLING
    assert backend.stop_calls == 1
    assert terminal.value.state is ExecutionState.CANCELLED
    assert after.error.code is DomainErrorCode.INVALID_EXECUTION_TRANSITION
    assert backend.stop_calls == 1


def test_terminal_timeout_stops_once_quarantines_and_rejects_reuse():
    async def scenario():
        backend = ExecutionBackend(outcome=BackendExecutionOutcome.PREEMPTED)
        service, _ = fixture(
            backend,
            execution_timeout_max_seconds=0.05,
            execution_timeout_margin_seconds=0.0,
        )
        terminal = await service.execute_motion_plan("plan")
        later = await service.execute_motion_plan("plan")
        return backend, terminal, later

    backend, terminal, later = asyncio.run(scenario())
    assert terminal.value.state is ExecutionState.TIMED_OUT
    assert terminal.value.backend_stop_requested is True
    assert backend.stop_calls == 1
    assert backend.quarantined is True
    assert later.error.code is DomainErrorCode.BACKEND_QUARANTINED
