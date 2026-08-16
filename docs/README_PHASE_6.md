# Phase 6 - Live MoveIt Integration and OpenMANIPULATOR-X Baseline

## Scope and outcome

Phase 6 verified the existing Phase 5 adapter against a real, nonphysical ROS 2 Jazzy / MoveIt graph and added a reusable immutable OpenMANIPULATOR-X descriptor. It did not add MCP capabilities, trajectory execution, physical-robot access, or controller management.

```text
ManipulatorService
 -> SafetyEvaluator
 -> JazzyManipulatorAdapter
 -> MoveIt 2.12.4 services
 -> official OpenMANIPULATOR-X 4.1.3 model
 -> ros2_control mock_components/GenericSystem
```

All Phase 5 adapter operations worked without correction. The generic adapter remains manipulator-independent; model-specific names live only in `profiles/open_manipulator_x.py`.

## Installed baseline and package audit

The initial audit found MoveIt 2.12.4, ros2_control 4.45.2, ros2_controllers 4.40.1, Gazebo/`ros_gz`, and `gz_ros2_control`, but no OpenMANIPULATOR package. After explicit approval, these official Jazzy binaries were installed:

| Package | Installed version |
|---|---|
| `ros-jazzy-open-manipulator-moveit-config` | `4.1.3-1noble.20260617.162003` |
| `ros-jazzy-open-manipulator-bringup` | `4.1.3-1noble.20260616.082337` |
| `ros-jazzy-open-manipulator-description` | `4.1.3-1noble.20260616.074531` |
| `ros-jazzy-dynamixel-hardware-interface` | `1.5.2-1noble.20260615.160408` |
| `ros-jazzy-dynamixel-interfaces` | `1.0.1-1noble.20260615.111021` |
| `ros-jazzy-dynamixel-sdk` | `4.0.3-1noble.20260225.231459` |

The two requested packages pulled the remaining four dependencies. No Python dependency changed. Gazebo was not needed because the official bringup supports ros2_control mock hardware.

## Verified model and semantic profile

The installed configuration matches the functional files in the official ROBOTIS `4.1.3` tag. The verified descriptor is:

| Property | Value |
|---|---|
| Robot/model name | `open_manipulator_x` |
| Model/planning frame | `world` |
| Arm group | `arm` |
| Active arm joints | `joint1`, `joint2`, `joint3`, `joint4` |
| Arm links | `link1` through `link5`, `end_effector_link` |
| Arm base link | `link1` |
| End-effector link/tool frame | `end_effector_link` |
| SRDF end effector | `end_effector`, parent link `end_effector_link`, group `arm` |
| Gripper group | `gripper` |
| Gripper joints | `gripper_left_joint`, `gripper_right_joint` |
| Gripper links | `gripper_left_link`, `gripper_right_link` (based at `link5`) |

The SRDF arm group also names fixed `end_effector_joint`. The backend-neutral descriptor lists active/plannable arm joints, not that fixed joint. The URDF makes `gripper_right_joint` mimic `gripper_left_joint`; the live `/joint_states` sample publishes the left gripper joint and four arm joints, while MoveIt robot-state responses may contain both gripper joints.

The installed `kinematics.yaml` selects `kdl_kinematics_plugin/KDLKinematicsPlugin` for `arm`, with search resolution `0.005`, timeout `0.005`, and `position_only_ik: true`. The launch loaded OMPL as its context pipeline and also loaded configured Pilz, CHOMP, and STOMP pipelines. No planner ID or pipeline ID was forced by the adapter.

## Deterministic live baseline

The baseline used ROS domain `66`, a temporary ROS home/log directory, no RViz, no initialization motion, and official launch files:

```bash
export ROS_DOMAIN_ID=66
export ROS_LOG_DIR=/tmp/ros2_manipulator_mcp_phase6_logs
export HOME=/tmp/ros2_manipulator_mcp_phase6_home

ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
  use_mock_hardware:=true init_position:=false start_rviz:=false

ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py \
  start_rviz:=false \
  warehouse_sqlite_path:=/tmp/ros2_manipulator_mcp_phase6_home/warehouse.sqlite
```

The bringup loaded `mock_components/GenericSystem`, robot_state_publisher, the joint-state broadcaster, and the official controllers. These controllers were started by the vendor launch as runtime infrastructure only; this project did not call, manage, switch, or expose them.

## Live graph audit

The live graph contained `move_group`, `controller_manager`, `robot_state_publisher`, `joint_state_broadcaster`, the arm/gripper controller nodes, and supporting internal MoveIt nodes.

Relevant observed services matched every Phase 5 default exactly:

| Service | Type |
|---|---|
| `/compute_ik` | `moveit_msgs/srv/GetPositionIK` |
| `/compute_fk` | `moveit_msgs/srv/GetPositionFK` |
| `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` |
| `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` |
| `/plan_kinematic_path` | `moveit_msgs/srv/GetMotionPlan` |
| `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` |

Relevant observed topics included `/joint_states`, `/robot_description`, `/robot_description_semantic`, `/planning_scene`, `/monitored_planning_scene`, `/collision_object`, `/tf`, and `/tf_static`. `/joint_states` was `sensor_msgs/msg/JointState`, reliable, volatile, keep-last depth 1. The adapter's best-effort sensor-data subscription is DDS-compatible with that publisher and received live state deterministically.

Observed actions included `/move_action` (`moveit_msgs/action/MoveGroup`), `/execute_trajectory` (`moveit_msgs/action/ExecuteTrajectory`), `/arm_controller/follow_joint_trajectory`, `/gripper_controller/gripper_cmd`, and `/sequence_move_group`. They were audited only. None is called or exposed by this project.

TF topics were present and robot_state_publisher initialized the model. Live FK returned `end_effector_link` in `world`, confirming the required transform/model chain for adapter operations.

## Adapter interoperability results

The existing adapter succeeded live for:

- current `JointState` acquisition
- FK to `end_effector_link` in `world`
- collision-aware IK with a supplied seed state
- state validity for `arm`
- plan-only joint-goal planning
- plan-only pose-goal planning
- Cartesian path computation with a reported completion fraction and trajectory
- planning-scene read
- box primitive add, subsequent read, and remove

The live probe observed a Cartesian fraction of `0.6666666666666666` for its two close waypoints. This was preserved as backend output rather than rounded or treated as complete. The integration policy explicitly requires at least `0.5` for that deterministic test scenario.

No Phase 5 defect was found. No Phase 5 implementation or test was removed or altered.

## Descriptor mechanism

No documented generic MoveIt Python discovery API was found or invented. Parsing private MoveIt internals was rejected. Phase 6 therefore uses an explicit domain-only profile validated against the installed URDF, SRDF, kinematics configuration, controller configuration, and live graph. Future deployments can supply a different `ManipulatorDescriptor` to the unchanged generic adapter.

## Integration tests

`tests/integration/test_open_manipulator_x_live.py` contains three opt-in tests:

1. Descriptor, current state, FK, IK, and validity prove the profile and core model interoperability together.
2. Joint, pose, and Cartesian planning prove that all planning paths pass through `ManipulatorService` and `SafetyEvaluator` before reaching MoveIt.
3. Primitive add/read/remove proves a state-changing scene operation passes through the same application policy boundary and round-trips through the live planning scene.

The tests require the documented graph and `ROS2_MANIPULATOR_MCP_RUN_LIVE_TESTS=1`; otherwise they skip rather than create timing-dependent failures on systems without ROS. The live Phase 6 run reported `3 passed, 25 deselected`.

## Authoritative source audit

Sources consulted:

- installed package manifests, launch files, URDF/Xacro, SRDF, `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`, planning-pipeline YAML, and ros2_control YAML under `/opt/ros/jazzy`
- installed live ROS graph (`ros2 node/service/topic/action` introspection and topic endpoint QoS)
- [official ROBOTIS OpenMANIPULATOR source, tag 4.1.3](https://github.com/ROBOTIS-GIT/open_manipulator/tree/4.1.3)
- [official ROBOTIS Jazzy branch](https://github.com/ROBOTIS-GIT/open_manipulator/tree/jazzy), audited at commit `1b741c0839d99a7c53cc6abf7f5a6c3990ecae3a`
- the Phase 5 primary MoveIt 2.12.4, `moveit_msgs` 2.6.0, and `rclpy` 7.1.11 sources recorded in `README_PHASE_5.md`

Exact versions verified were Ubuntu 24.04.4, Python 3.12.3, ROS 2 Jazzy, rclpy 7.1.11, MoveIt 2.12.4, `moveit_msgs` 2.6.0, and OpenMANIPULATOR 4.1.3 binary packages. The official tag resolves to commit `86163c4fbc7d8aeee3cb05d5733cc589c51299ae`; installed functional configuration matched that tag.

Discrepancies and runtime observations:

- `MoveItConfigsBuilder` warns that it cannot infer root-level `config/open_manipulator_x.urdf` or `.srdf`; the vendor launch explicitly supplies the nested SRDF and obtains `robot_description` from the published topic, and `move_group` starts successfully.
- MoveIt logs `Could not identify parent group for end-effector 'end_effector'` for the vendor SRDF definition. Core arm FK, IK, planning, and pose planning still succeeded. The project does not rewrite vendor semantics.
- The vendor mock launch starts controllers even though Phase 6 never invokes execution. This is required by that official bringup, not controller-management functionality added here.
- CycloneDDS local graph discovery requires normal host network-interface access. Sandboxed runs that deny interface enumeration cannot launch ROS nodes.
- The installed gripper controller reports an upstream deprecation warning for `position_controllers/GripperActionController`; it is outside this project's ownership and execution is out of scope.
- One simultaneous Ctrl-C shutdown emitted ros2_control PAL statistics publisher `context ... invalid` messages after hardware/controller deactivation; the managed nodes still exited and this did not involve adapter-owned resources.

Nothing required for the implemented Phase 6 profile or live service paths remained unverifiable.

## Files changed

Created:

- `src/ros2_manipulator_mcp/profiles/__init__.py`
- `src/ros2_manipulator_mcp/profiles/open_manipulator_x.py`
- `tests/integration/test_open_manipulator_x_live.py`
- `docs/README_PHASE_6.md`

Updated:

- `pyproject.toml` to register the opt-in `integration` test marker
- `docs/README_PHASES.md` to record Phase 6 completion

## Limitations and phase boundary

- The profile is explicit, not runtime-generated from arbitrary URDF/SRDF.
- Prefixes and namespaces require a separate deployment-specific descriptor/settings pair.
- The baseline is mock hardware, not Gazebo physics and not a physical robot.
- No execution API, controller access, physical safety claim, attached object, mesh, Servo, Hybrid Planning, task construction, perception, navigation, or orchestration was added.
- No MCP Tool, Resource, or Prompt was added.

Phase 7 was not started.
