"""Register concise advisory workflows over existing MCP capabilities."""

from typing import Annotated

from mcp.server import MCPServer
from pydantic import Field


def register_prompts(server: MCPServer) -> None:
    """Register Phase 9 prompts without invoking application operations."""

    @server.prompt(
        name="inspect_manipulator",
        description="Inspect manipulator readiness and context without changing state.",
    )
    async def inspect_manipulator(
        focus: Annotated[
            str | None,
            Field(description="Optional inspection focus or concern."),
        ] = None,
    ) -> str:
        detail = f" Give extra attention to: {focus}." if focus else ""
        return (
            "Inspect the manipulator without changing state. Read "
            "manipulator://overview, manipulator://health, "
            "manipulator://state/current, manipulator://groups, and "
            "manipulator://scene. Summarize identity, available planning groups, "
            "state timestamp availability, scene revision, and readiness. Do not "
            "plan, mutate the scene, or execute motion. Physical execution is not "
            f"available in v0.1.0.{detail}"
        )

    @server.prompt(
        name="diagnose_kinematics_failure",
        description="Diagnose FK or IK failure using read-only context first.",
    )
    async def diagnose_kinematics_failure(
        failure: Annotated[str, Field(description="Observed FK/IK error or symptom.")],
        planning_group: Annotated[
            str | None, Field(description="Planning group under investigation.")
        ] = None,
        target_link: Annotated[
            str | None, Field(description="Target link under investigation.")
        ] = None,
        frame: Annotated[
            str | None, Field(description="Pose or computation frame under investigation.")
        ] = None,
    ) -> str:
        context = _provided_context(planning_group, target_link, frame)
        return (
            f"Diagnose this kinematics failure: {failure}. {context} Read "
            "manipulator://health, manipulator://overview, "
            "manipulator://groups, and manipulator://state/current first. Verify "
            "the selected group, link, frame, finite joint values, normalized "
            "quaternion, tolerances, and seed-state assumptions. Use "
            "compute_forward_kinematics or compute_inverse_kinematics only if a "
            "targeted diagnostic call is needed. Do not plan, change the scene, "
            "or execute motion."
        )

    @server.prompt(
        name="diagnose_planning_failure",
        description="Diagnose planning failure without automatically replanning.",
    )
    async def diagnose_planning_failure(
        failure: Annotated[str, Field(description="Observed planning error or symptom.")],
        goal_summary: Annotated[
            str | None, Field(description="Optional joint or pose goal summary.")
        ] = None,
        plan_id: Annotated[
            str | None, Field(description="Optional server-owned plan identifier.")
        ] = None,
    ) -> str:
        goal = goal_summary or "No goal summary was supplied."
        plan = (
            f" Review manipulator://plans/{plan_id}." if plan_id
            else " No stored plan identifier was supplied."
        )
        return (
            f"Diagnose this planning failure: {failure}. Goal context: {goal}.{plan} "
            "Read manipulator://health, manipulator://safety, "
            "manipulator://state/current, manipulator://groups, and "
            "manipulator://scene. Check policy limits, group/goal consistency, "
            "start-state validity, frames, tolerances, scaling, collision objects, "
            "and scene revision. Explain likely corrections, but do not "
            "automatically replan, mutate the scene, or execute motion."
        )

    @server.prompt(
        name="review_motion_plan",
        description="Review one stored motion plan and its validation context.",
    )
    async def review_motion_plan(
        plan_id: Annotated[str, Field(description="Server-owned motion plan identifier.")],
    ) -> str:
        return (
            f"Review manipulator://plans/{plan_id} and manipulator://safety. "
            "Summarize the goal, planning group, scene revision, policy identity, "
            "planning duration, scaling, bounded trajectory metadata, and current "
            "validation status. If needed, compare get_motion_plan and "
            "validate_motion_plan results. Do not execute or discard the plan. "
            "Validation does not establish certified physical safety, and physical "
            "execution is unavailable in v0.1.0."
        )

    @server.prompt(
        name="review_planning_scene_change",
        description="Review an intended primitive scene change without applying it.",
    )
    async def review_planning_scene_change(
        intended_change: Annotated[
            str, Field(description="Proposed primitive add, replace, or remove operation.")
        ],
        object_id: Annotated[
            str | None, Field(description="Existing or proposed collision-object ID.")
        ] = None,
    ) -> str:
        existing = (
            f" Read manipulator://scene/objects/{object_id} if it exists."
            if object_id else " No object ID was supplied."
        )
        return (
            f"Review this intended planning-scene change without applying it: "
            f"{intended_change}.{existing} Read manipulator://scene, "
            "manipulator://scene/objects, and manipulator://safety. Check object "
            "naming, primitive type and dimension ordering, pose frame, replacement "
            "intent, duplicates, and configured limits. Explain whether the proposed "
            "apply_collision_object or remove_collision_object call appears policy-"
            "compatible, but do not call either tool."
        )

    @server.prompt(
        name="safe_manipulation_workflow",
        description="Guide an inspect-plan-review-validate workflow that stops before execution.",
    )
    async def safe_manipulation_workflow(
        goal_summary: Annotated[
            str, Field(description="Desired joint or pose planning outcome.")
        ],
        planning_group: Annotated[
            str | None, Field(description="Optional planning-group identifier.")
        ] = None,
    ) -> str:
        group = planning_group or "a group selected from manipulator://groups"
        return (
            f"Advisory workflow for goal: {goal_summary}. Use {group}. "
            "1) Inspect manipulator://overview, manipulator://health, "
            "manipulator://safety, manipulator://state/current, and "
            "manipulator://scene. 2) Confirm group, joints/links, frame, goal, and "
            "policy limits. 3) Request plan_to_joint_goal or plan_to_pose_goal, or "
            "compute_cartesian_path when appropriate. 4) Review the returned plan_id "
            "through manipulator://plans/{plan_id} or get_motion_plan. 5) Call "
            "validate_motion_plan and explain its policy/scene context. 6) STOP. Do "
            "not execute motion. Physical execution is not exposed in v0.1.0, and "
            "policy validation is not certified physical safety."
        )


def _provided_context(
    planning_group: str | None,
    target_link: str | None,
    frame: str | None,
) -> str:
    values = []
    if planning_group:
        values.append(f"planning group={planning_group}")
    if target_link:
        values.append(f"target link={target_link}")
    if frame:
        values.append(f"frame={frame}")
    return "Provided context: " + (", ".join(values) if values else "none") + "."
