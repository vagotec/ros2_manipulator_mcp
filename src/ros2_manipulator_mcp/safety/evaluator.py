"""Backend-neutral deterministic policy evaluation."""

from ros2_manipulator_mcp.domain import (
    CartesianPathRequest,
    CartesianPathResult,
    CollisionObject,
    JointGoal,
    MotionPlan,
    PlanningRequest,
    PlanningSceneSnapshot,
    PoseGoal,
)
from ros2_manipulator_mcp.safety.models import (
    SafetyDecision,
    SafetyFinding,
    SafetyPolicy,
    SafetySeverity,
)


class SafetyEvaluator:
    """Evaluate application operations without backend access or mutation."""

    def __init__(self, policy: SafetyPolicy | None = None) -> None:
        self.policy = policy or SafetyPolicy()

    def evaluate_planning_request(
        self,
        request: PlanningRequest,
    ) -> SafetyDecision:
        """Validate a joint or pose planning request against policy."""
        findings: list[SafetyFinding] = []
        group_name = request.goal.group_name
        self._check_group(group_name, findings)
        self._maximum(
            request.allowed_planning_time_seconds,
            self.policy.max_planning_time_seconds,
            "PLANNING_TIME_EXCEEDED",
            "Planning time exceeds the configured maximum.",
            findings,
        )
        self._maximum(
            request.planning_attempts,
            self.policy.max_planning_attempts,
            "PLANNING_ATTEMPTS_EXCEEDED",
            "Planning attempts exceed the configured maximum.",
            findings,
        )
        self._maximum(
            request.max_velocity_scaling_factor,
            self.policy.max_velocity_scaling_factor,
            "VELOCITY_SCALING_EXCEEDED",
            "Velocity scaling exceeds the configured maximum.",
            findings,
        )
        self._maximum(
            request.max_acceleration_scaling_factor,
            self.policy.max_acceleration_scaling_factor,
            "ACCELERATION_SCALING_EXCEEDED",
            "Acceleration scaling exceeds the configured maximum.",
            findings,
        )
        if (
            isinstance(request.goal, JointGoal)
            and len(request.goal.joint_names) > self.policy.max_joint_targets
        ):
            self._error(
                "JOINT_TARGET_LIMIT_EXCEEDED",
                "Joint target count exceeds the configured maximum.",
                findings,
            )
        if (
            isinstance(request.goal, PoseGoal)
            and self.policy.workspace is not None
            and not self.policy.workspace.contains(request.goal.pose.position)
        ):
            self._error(
                "POSE_OUTSIDE_WORKSPACE",
                "The pose goal lies outside configured workspace bounds.",
                findings,
            )
        return self._decision(findings)

    def evaluate_cartesian_request(
        self,
        request: CartesianPathRequest,
    ) -> SafetyDecision:
        """Validate Cartesian request limits and collision policy."""
        findings: list[SafetyFinding] = []
        self._check_group(request.group_name, findings)
        if len(request.waypoints) > self.policy.max_cartesian_waypoints:
            self._error(
                "CARTESIAN_WAYPOINT_LIMIT_EXCEEDED",
                "Waypoint count exceeds the configured maximum.",
                findings,
            )
        if not (
            self.policy.min_cartesian_step
            <= request.max_step
            <= self.policy.max_cartesian_step
        ):
            self._error(
                "CARTESIAN_STEP_OUT_OF_POLICY",
                "Cartesian step is outside configured bounds.",
                findings,
            )
        if self.policy.require_collision_avoidance and not request.avoid_collisions:
            self._error(
                "COLLISION_AVOIDANCE_REQUIRED",
                "Cartesian paths must enable collision avoidance.",
                findings,
            )
        self._maximum(
            request.max_velocity_scaling_factor,
            self.policy.max_velocity_scaling_factor,
            "VELOCITY_SCALING_EXCEEDED",
            "Velocity scaling exceeds the configured maximum.",
            findings,
        )
        self._maximum(
            request.max_acceleration_scaling_factor,
            self.policy.max_acceleration_scaling_factor,
            "ACCELERATION_SCALING_EXCEEDED",
            "Acceleration scaling exceeds the configured maximum.",
            findings,
        )
        if self.policy.workspace is not None:
            for waypoint in request.waypoints:
                if not self.policy.workspace.contains(waypoint.position):
                    self._error(
                        "WAYPOINT_OUTSIDE_WORKSPACE",
                        "A Cartesian waypoint lies outside workspace bounds.",
                        findings,
                    )
                    break
        return self._decision(findings)

    def evaluate_cartesian_result(
        self,
        result: CartesianPathResult,
    ) -> SafetyDecision:
        """Reject path results below the configured completion fraction."""
        findings: list[SafetyFinding] = []
        if result.fraction < self.policy.min_cartesian_completion_fraction:
            self._error(
                "CARTESIAN_PATH_INCOMPLETE",
                "Cartesian path completion is below the configured minimum.",
                findings,
            )
        return self._decision(findings)

    def evaluate_collision_object(
        self,
        collision_object: CollisionObject,
        scene: PlanningSceneSnapshot,
        *,
        replace: bool,
    ) -> SafetyDecision:
        """Validate a primitive scene add or replacement."""
        findings: list[SafetyFinding] = []
        self._check_object_id(collision_object.object_id, findings)
        existing_ids = {item.object_id for item in scene.collision_objects}
        exists = collision_object.object_id in existing_ids
        if replace and not self.policy.allow_object_replace:
            self._error(
                "OBJECT_REPLACE_NOT_ALLOWED",
                "Object replacement is disabled by policy.",
                findings,
            )
        if exists and not replace:
            self._error(
                "OBJECT_ALREADY_EXISTS",
                "The object already exists; replacement must be explicit.",
                findings,
            )
        if not exists and len(existing_ids) >= self.policy.max_collision_objects:
            self._error(
                "COLLISION_OBJECT_LIMIT_EXCEEDED",
                "The planning scene has reached its configured object limit.",
                findings,
            )
        if (
            collision_object.primitive.primitive_type
            not in self.policy.allowed_primitive_types
        ):
            self._error(
                "PRIMITIVE_TYPE_NOT_ALLOWED",
                "The collision primitive type is not allowed by policy.",
                findings,
            )
        if any(
            value > self.policy.max_primitive_dimension
            for value in collision_object.primitive.dimensions
        ):
            self._error(
                "PRIMITIVE_DIMENSION_EXCEEDED",
                "A primitive dimension exceeds the configured maximum.",
                findings,
            )
        if (
            self.policy.allowed_scene_frames
            and collision_object.pose.frame_id not in self.policy.allowed_scene_frames
        ):
            self._error(
                "SCENE_FRAME_NOT_ALLOWED",
                "The collision object frame is not allowed by policy.",
                findings,
            )
        return self._decision(findings)

    def evaluate_collision_object_removal(
        self,
        object_id: str,
    ) -> SafetyDecision:
        """Validate removal naming policy without claiming object existence."""
        findings: list[SafetyFinding] = []
        self._check_object_id(object_id, findings)
        return self._decision(findings)

    def evaluate_stored_plan(
        self,
        plan: MotionPlan,
        *,
        associated_policy_id: str,
    ) -> SafetyDecision:
        """Validate stored metadata available without a live backend."""
        findings: list[SafetyFinding] = []
        if associated_policy_id != self.policy.policy_id:
            self._error(
                "PLAN_POLICY_MISMATCH",
                "The plan was created under a different safety policy.",
                findings,
            )
        self._check_group(plan.request.goal.group_name, findings)
        if self.policy.require_scene_revision and not plan.scene_revision.strip():
            self._error(
                "PLAN_SCENE_REVISION_REQUIRED",
                "The stored plan has no planning-scene revision.",
                findings,
            )
        return self._decision(findings)

    def _check_group(
        self,
        group_name: str,
        findings: list[SafetyFinding],
    ) -> None:
        if (
            self.policy.allowed_planning_groups
            and group_name not in self.policy.allowed_planning_groups
        ):
            self._error(
                "PLANNING_GROUP_NOT_ALLOWED",
                "The planning group is not allowed by policy.",
                findings,
            )

    def _check_object_id(
        self,
        object_id: str,
        findings: list[SafetyFinding],
    ) -> None:
        prefixes = self.policy.allowed_object_id_prefixes
        if prefixes and not object_id.startswith(prefixes):
            self._error(
                "OBJECT_ID_NOT_ALLOWED",
                "The collision object identifier is not allowed by policy.",
                findings,
            )

    @staticmethod
    def _maximum(
        value: float,
        maximum: float,
        code: str,
        message: str,
        findings: list[SafetyFinding],
    ) -> None:
        if value > maximum:
            SafetyEvaluator._error(code, message, findings)

    @staticmethod
    def _error(
        code: str,
        message: str,
        findings: list[SafetyFinding],
    ) -> None:
        findings.append(SafetyFinding(code, message, SafetySeverity.ERROR))

    def _decision(self, findings: list[SafetyFinding]) -> SafetyDecision:
        return SafetyDecision(self.policy.policy_id, tuple(findings))
