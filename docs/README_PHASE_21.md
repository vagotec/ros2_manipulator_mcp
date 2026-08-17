# Phase 21 - v0.2.0 Final Audit and Release Readiness

## Scope and outcome

Phase 21 performed the final source, version, architecture, MCP-surface,
execution-contract, regression, and documentation audit for development version
`0.2.0`. It introduced no new capability and used no physical hardware or live
ROS graph. The final classification is:

**B - READY FOR v0.2.0 RELEASE WITH DOCUMENTED LIMITATIONS.**

The accepted limitations are already part of the declared v0.2.0 contract and
do not invalidate the implemented planning, execution, or mock-cancellation
claims.

## Git and worktree audit

The audit ran on branch `dev`, tracking `origin/dev`, at HEAD
`f016a6e4bc3e1489502e7c6213828e1bded8e9f8`. The initial Phase 21 worktree
contained the accepted Phase 20 documentation and gripper mimic-joint correction:

- modified `docs/README_PHASES.md`;
- modified `src/ros2_manipulator_mcp/profiles/open_manipulator_x.py`;
- modified `tests/application/test_pre_execution_validation.py`;
- untracked `docs/README_PHASE_20.md`;
- untracked `tests/profiles/test_open_manipulator_x.py`.

Phase 21 retained those changes and added only release documentation and stale
version-language corrections listed below. Repository-local `__pycache__`,
`.pytest_cache`, and virtual-environment files are ignored development artifacts;
no accidental generated file is tracked or newly exposed by `git status`.
Phase-specific helper scripts and source mirrors under `/tmp` remain outside the
repository. `git diff --check` passed.

## Version audit

Version `0.2.0` agrees across `pyproject.toml`, the root project entry in
`uv.lock`, `ros2_manipulator_mcp.__version__`, the MCP server version, and the
version assertions in the foundation and MCP stdio tests. The README now
describes the v0.2.0 execution surface instead of the superseded v0.1.0
planning-only surface. Historical phase references to the immutable v0.1.0
release remain intentionally unchanged.

## Production-change and architecture audit

The accepted Phase 14-20 implementation retains:

- backend-neutral execution records, transitions, registry, and one-active-
  execution ownership;
- atomic plan reservation, release before backend acceptance, and permanent
  consumption after acceptance;
- fresh-state, start-tolerance, policy/group, and exact scene-revision
  revalidation immediately before submission;
- bounded MoveIt `ExecuteTrajectory` submission, terminal-result handling,
  timeout stop attempt, and quarantine of ambiguous or unsafe outcomes;
- MoveIt 2.12.4's audited `stop` event on `/trajectory_execution_event`, hidden
  behind the Jazzy adapter;
- bounded MCP execution tools, resource, and diagnostic prompt;
- fail-closed default execution configuration;
- the OpenMANIPULATOR-X profile correction that exposes only independently
  commandable `gripper_left_joint` while retaining the right mimic link.

AST import-boundary inspection passed for `domain`, `application`, and `safety`
with no ROS, MoveIt, or MCP imports, and for `mcp` with no ROS or MoveIt imports.
Version-specific messages, actions, runtime calls, and the stop topic remain
under `ros/jazzy`. The public API contains no direct `FollowJointTrajectory`, raw
trajectory, controller-name, arbitrary ROS service/action/topic, arbitrary
shell, or controller-management input.

## MCP surface

The final registered surface is exactly:

- 22 Tools;
- 7 static Resources;
- 4 Resource Templates;
- 7 Prompts.

The execution subset is exactly `execute_motion_plan(plan_id)`,
`get_execution_status(execution_id)`, `cancel_execution(execution_id)`,
`manipulator://executions/{execution_id}`, and
`diagnose_execution_failure(execution_id)`.

## Safety and execution contract

The packaged `default.toml` retains `execution.enabled = false`. Execution
accepts only an application-owned `plan_id`. A reserved plan cannot be reused
concurrently; pre-acceptance failures release it, while backend acceptance
consumes it permanently. Fresh-state age, start-state tolerance, and exact
scene-revision equality are enforced. Only one execution may be active.

Timeout attempts the narrow adapter stop and quarantines the backend where the
outcome is unsafe or ambiguous. Ambiguous cancellation also quarantines. Neither
application nor adapter quarantine silently clears; process/adapter restart is
required. `cancel_execution` is not an emergency stop or certified
machinery-safety feature. `CANCELLED` requires causal MoveIt `PREEMPTED` plus
bounded state stabilization, but does not establish certified physical
standstill. Physical power cutoff remains the emergency mechanism.

## Final non-hardware verification

Phase 21 performed one final completed graph-independent regression run. Its
recorded commands and outcomes are:

- `.venv/bin/pytest -q -m 'not integration'`: `67 passed, 9 deselected` in
  `3.17 s`;
- `.venv/bin/python -m compileall -q src tests`: passed;
- AST import-boundary check: passed;
- `UV_CACHE_DIR=/tmp/ros2_manipulator_mcp_phase21_uv_cache uv lock --check`:
  passed (`Resolved 39 packages in 5 ms`);
- `git diff --check`: passed.

Opt-in live mock tests were not repeated: the established Phase 17-19 mock
execution/cancellation evidence remains authoritative, and no cancellation
implementation changed in Phase 21.

An initial sandboxed invocation was terminated after the MCP SDK's synchronous
Resource read blocked in `anyio.to_thread`; an isolated AnyIO thread probe
reproduced that sandbox restriction. No test assertion failed. The completed
suite was therefore run outside that thread-restricted sandbox with all nine
`integration`-marked tests explicitly deselected. It used no ROS graph or
hardware. A temporary attempt to make one Resource callback asynchronous was
reverted after identifying the environmental cause and is not part of the
release-candidate diff.

## Inherited Phase 20 hardware evidence

No hardware verification was repeated. Phase 20 remains authoritative for real
OpenMANIPULATOR-X startup, IDs 11-15, the real Dynamixel plugin, controller and
Torque ON/OFF lifecycle, fresh state, execution and separately visible movement
for joints 1-4 and ID 15, the real
`MCP -> application -> MoveIt -> ros2_control -> Dynamixel` path, orderly
deactivation, Torque OFF for all five IDs, and serial release.

Accepted v0.2.0 limitations are:

1. mock-hardware cancellation is verified, while real-hardware cancellation is
   deliberately not physically release-verified;
2. the tested ID 15 mechanism has a mechanical/encoder-reference mismatch
   relative to the official model;
3. the initiating cause of one historical ID 12 shutdown remains unknown;
4. FastSyncRead can report `-3001` before the official driver successfully
   falls back to normal SyncRead.

## Files changed through the final audit

The release-candidate worktree contains the accepted Phase 20 changes plus:

- `README.md`: updated from the v0.1.0 planning-only description to the complete
  v0.2.0 surface, workflows, safety contract, evidence, and limitations;
- `pyproject.toml`: updated the package description to include bounded
  execution; the version and dependencies are unchanged;
- `docs/README_PHASES.md`: indexed Phases 20 and 21;
- `docs/README_PHASE_21.md`: this final audit record;
- `src/ros2_manipulator_mcp/mcp/prompts/registration.py`: removed three stale
  claims that execution was unavailable in v0.1.0 while preserving the prompts'
  non-execution instructions;
- `src/ros2_manipulator_mcp/domain/scene.py`,
  `src/ros2_manipulator_mcp/safety/models.py`, and
  `src/ros2_manipulator_mcp/ros/jazzy/adapter.py`: removed stale v0.1.0 labels
  from docstrings only.

No hardware, Dynamixel bus, torque, physical trajectory, or live real-hardware
runtime was used. No commit, push, merge, tag, package publication, or release
was performed.
