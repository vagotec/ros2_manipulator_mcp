"""Conversions between domain values and installed Jazzy message types."""

from math import floor

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose as RosPose
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    CollisionObject as RosCollisionObject,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState as RosRobotState,
    RobotTrajectory,
)
from sensor_msgs.msg import JointState as RosJointState
from shape_msgs.msg import SolidPrimitive

from ros2_manipulator_mcp.domain import (
    CollisionObject,
    CollisionPrimitive,
    JointGoal,
    JointState,
    PlanningRequest,
    PlanningSceneSnapshot,
    Pose,
    PoseGoal,
    PrimitiveType,
    Quaternion,
    RobotState,
    Trajectory,
    TrajectoryPoint,
    Vector3,
)


_PRIMITIVE_TO_ROS = {
    PrimitiveType.BOX: SolidPrimitive.BOX,
    PrimitiveType.SPHERE: SolidPrimitive.SPHERE,
    PrimitiveType.CYLINDER: SolidPrimitive.CYLINDER,
    PrimitiveType.CONE: SolidPrimitive.CONE,
}
_PRIMITIVE_FROM_ROS = {value: key for key, value in _PRIMITIVE_TO_ROS.items()}


def to_duration(seconds: float) -> Duration:
    """Convert finite non-negative seconds to a normalized ROS duration."""
    whole_seconds = floor(seconds)
    nanoseconds = round((seconds - whole_seconds) * 1_000_000_000)
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def from_joint_state_message(message: RosJointState) -> JointState:
    """Convert a Jazzy JointState message to a validated domain state."""
    return JointState(
        joint_names=tuple(message.name),
        positions=tuple(message.position),
        velocities=tuple(message.velocity) if message.velocity else None,
        efforts=tuple(message.effort) if message.effort else None,
    )


def to_joint_state_message(state: JointState) -> RosJointState:
    """Convert a domain joint state to the installed Jazzy message."""
    message = RosJointState()
    message.name = list(state.joint_names)
    message.position = list(state.positions)
    message.velocity = list(state.velocities or ())
    message.effort = list(state.efforts or ())
    return message


def from_robot_state_message(message: RosRobotState) -> RobotState:
    """Convert the single-DOF portion of a MoveIt robot state."""
    stamp = message.joint_state.header.stamp
    timestamp = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000
    return RobotState(
        joints=from_joint_state_message(message.joint_state),
        timestamp_seconds=timestamp,
    )


def to_robot_state_message(
    state: RobotState | None,
    *,
    is_diff: bool,
) -> RosRobotState:
    """Convert a domain state or an empty current-state diff marker."""
    message = RosRobotState()
    message.is_diff = is_diff
    if state is not None:
        message.joint_state = to_joint_state_message(state.joints)
    return message


def to_pose_message(pose: Pose) -> RosPose:
    """Convert domain pose coordinates without changing their frame."""
    message = RosPose()
    message.position.x = pose.position.x
    message.position.y = pose.position.y
    message.position.z = pose.position.z
    message.orientation.x = pose.orientation.x
    message.orientation.y = pose.orientation.y
    message.orientation.z = pose.orientation.z
    message.orientation.w = pose.orientation.w
    return message


def to_pose_stamped_message(pose: Pose) -> PoseStamped:
    """Convert a framed domain pose."""
    message = PoseStamped()
    message.header.frame_id = pose.frame_id
    message.pose = to_pose_message(pose)
    return message


def from_pose_message(message: RosPose, frame_id: str) -> Pose:
    """Convert a ROS pose expressed in an explicit frame."""
    return Pose(
        frame_id=frame_id,
        position=Vector3(
            message.position.x,
            message.position.y,
            message.position.z,
        ),
        orientation=Quaternion(
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ),
    )


def from_trajectory_message(message: RobotTrajectory) -> Trajectory:
    """Convert the supported single-DOF joint trajectory."""
    joint_trajectory = message.joint_trajectory
    points = tuple(
        TrajectoryPoint(
            positions=tuple(point.positions),
            velocities=tuple(point.velocities) if point.velocities else None,
            accelerations=(
                tuple(point.accelerations) if point.accelerations else None
            ),
            time_from_start_seconds=(
                float(point.time_from_start.sec)
                + float(point.time_from_start.nanosec) / 1_000_000_000
            ),
        )
        for point in joint_trajectory.points
    )
    return Trajectory(tuple(joint_trajectory.joint_names), points)


def to_goal_constraints(
    request: PlanningRequest,
    *,
    joint_goal_tolerance: float,
) -> Constraints:
    """Build MoveIt goal constraints using verified 2.12.4 semantics."""
    if isinstance(request.goal, JointGoal):
        constraints = Constraints()
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=position,
                tolerance_above=joint_goal_tolerance,
                tolerance_below=joint_goal_tolerance,
                weight=1.0,
            )
            for name, position in zip(
                request.goal.joint_names,
                request.goal.positions,
            )
        ]
        return constraints

    goal: PoseGoal = request.goal
    constraints = Constraints()
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = goal.pose.frame_id
    position_constraint.link_name = goal.target_link
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [goal.position_tolerance]
    position_constraint.constraint_region.primitives = [sphere]
    position_constraint.constraint_region.primitive_poses = [
        _position_only_pose(goal.pose.position)
    ]
    position_constraint.weight = 1.0

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = goal.pose.frame_id
    orientation_constraint.link_name = goal.target_link
    orientation_constraint.orientation = to_pose_message(goal.pose).orientation
    orientation_constraint.absolute_x_axis_tolerance = goal.orientation_tolerance
    orientation_constraint.absolute_y_axis_tolerance = goal.orientation_tolerance
    orientation_constraint.absolute_z_axis_tolerance = goal.orientation_tolerance
    orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
    orientation_constraint.weight = 1.0
    constraints.position_constraints = [position_constraint]
    constraints.orientation_constraints = [orientation_constraint]
    return constraints


def to_collision_object_message(
    collision_object: CollisionObject,
    *,
    operation: int = RosCollisionObject.ADD,
) -> RosCollisionObject:
    """Convert one supported primitive object to a MoveIt scene operation."""
    message = RosCollisionObject()
    message.header.frame_id = collision_object.pose.frame_id
    message.id = collision_object.object_id
    message.operation = operation
    message.pose.orientation.w = 1.0
    primitive = SolidPrimitive()
    primitive.type = _PRIMITIVE_TO_ROS[
        collision_object.primitive.primitive_type
    ]
    primitive.dimensions = list(collision_object.primitive.dimensions)
    message.primitives = [primitive]
    message.primitive_poses = [to_pose_message(collision_object.pose)]
    return message


def removal_collision_object_message(object_id: str) -> RosCollisionObject:
    """Create the installed MoveIt REMOVE operation for one object ID."""
    message = RosCollisionObject()
    message.id = object_id
    message.operation = RosCollisionObject.REMOVE
    return message


def from_collision_object_message(message: RosCollisionObject) -> CollisionObject:
    """Convert exactly one supported primitive collision object."""
    if len(message.primitives) != 1:
        raise ValueError(
            f"Collision object '{message.id}' must contain exactly one primitive."
        )
    if len(message.primitive_poses) > 1:
        raise ValueError(
            f"Collision object '{message.id}' has ambiguous primitive poses."
        )
    if message.meshes or message.planes:
        raise ValueError(
            f"Collision object '{message.id}' contains unsupported geometry."
        )
    primitive_message = message.primitives[0]
    primitive_type = _PRIMITIVE_FROM_ROS.get(primitive_message.type)
    if primitive_type is None:
        raise ValueError(
            f"Collision object '{message.id}' uses an unsupported primitive."
        )
    object_pose = from_pose_message(message.pose, message.header.frame_id)
    if message.primitive_poses:
        local_pose = from_pose_message(
            message.primitive_poses[0],
            message.header.frame_id,
        )
        pose = _compose_pose(object_pose, local_pose)
    else:
        pose = object_pose
    return CollisionObject(
        object_id=message.id,
        primitive=CollisionPrimitive(
            primitive_type,
            tuple(primitive_message.dimensions),
        ),
        pose=pose,
    )


def from_planning_scene_message(
    message: PlanningScene,
    *,
    revision: str,
) -> PlanningSceneSnapshot:
    """Convert the primitive world and current single-DOF robot state."""
    objects = tuple(
        from_collision_object_message(item)
        for item in message.world.collision_objects
    )
    robot_state = None
    if message.robot_state.joint_state.name:
        robot_state = from_robot_state_message(message.robot_state)
    return PlanningSceneSnapshot(revision, objects, robot_state)


def _position_only_pose(position: Vector3) -> RosPose:
    message = RosPose()
    message.position.x = position.x
    message.position.y = position.y
    message.position.z = position.z
    message.orientation.w = 1.0
    return message


def _compose_pose(parent: Pose, child: Pose) -> Pose:
    """Compose primitive-local geometry into its object's framed pose."""
    rotated = _rotate(parent.orientation, child.position)
    return Pose(
        parent.frame_id,
        Vector3(
            parent.position.x + rotated.x,
            parent.position.y + rotated.y,
            parent.position.z + rotated.z,
        ),
        _multiply(parent.orientation, child.orientation),
    )


def _multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    return Quaternion(
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
    )


def _rotate(rotation: Quaternion, vector: Vector3) -> Vector3:
    tx = 2.0 * (rotation.y * vector.z - rotation.z * vector.y)
    ty = 2.0 * (rotation.z * vector.x - rotation.x * vector.z)
    tz = 2.0 * (rotation.x * vector.y - rotation.y * vector.x)
    return Vector3(
        vector.x + rotation.w * tx + rotation.y * tz - rotation.z * ty,
        vector.y + rotation.w * ty + rotation.z * tx - rotation.x * tz,
        vector.z + rotation.w * tz + rotation.x * ty - rotation.y * tx,
    )
