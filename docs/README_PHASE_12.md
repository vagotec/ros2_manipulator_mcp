# Phase 12 - Real OpenMANIPULATOR-X Hardware Verification

## Status

- Pre-hardware audit: **COMPLETE**
- Hardware detection and permissions: **COMPLETE**
- Real read-only state/kinematics verification: **COMPLETE**
- Real planning-only verification: **COMPLETE**
- Physical trajectory execution verification: **NOT PERFORMED**
- Physical trajectory commands issued: **NONE**

Post-release note (2026-08-16): the real-hardware read/kinematics and
planning-only checkpoints documented below were completed after the v0.1.0
release. The release-time Phase 13 classification remains a historical record.

Phase 12 verified the real OpenMANIPULATOR-X through the MCP planning boundary.
It did not test or authorize physical trajectory execution. Visible physical
response during torque engagement was not independently observable through the
terminal; sampled velocities were zero and measured positions were unchanged
during planning.

## Authoritative baseline

| Component | Installed version |
|---|---|
| Ubuntu | 24.04.4 LTS |
| Python | 3.12.3 |
| ROS 2 | Jazzy |
| `rclpy` | 7.1.11 |
| MoveIt | 2.12.4 |
| `moveit_msgs` | 2.6.0 |
| MCP Python SDK / `mcp-types` | 2.0.0 / 2.0.0 |
| OpenMANIPULATOR packages | 4.1.3 |
| Dynamixel hardware interface | 1.5.2 |
| Dynamixel SDK | 4.0.3 |

The installed official launch, ros2_control Xacro, controller configuration,
MoveIt configuration, URDF joint limits, and generated ROS interfaces were the
runtime authority.

## Detected hardware

The connected FTDI FT232H interface (`0403:6014`, serial `FT45B86O`) appeared
as `/dev/ttyUSB0`, with stable link:

```text
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT45B86O-if00-port0
```

The link resolved to `/dev/ttyUSB0`. User `sarvg` was in `dialout`, and both
paths were readable and writable. The official OpenMANIPULATOR-X configuration
matched the detected/default port and specified 1,000,000 baud and Dynamixel
IDs 11 through 15.

## Audited startup and lifecycle

The approved hardware runtime used ROS domain 67 and exactly:

```bash
ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
  use_mock_hardware:=false \
  init_position:=false \
  start_rviz:=false \
  port_name:=/dev/ttyUSB0
```

`init_position:=false` prevented the explicit initialization trajectory. It
did not prevent the hardware plugin from configuring devices, enabling torque,
or activating position controllers.

The port opened at 1,000,000 baud. IDs 11-15 all responded as XM430-W350
(model 1020). The driver synchronized measured positions into command state
before enabling torque, then activated the arm controller, gripper controller,
and joint-state broadcaster. Initial FastSyncRead failed ten times with error
`-3001`; the driver fell back to normal SyncRead and remained operational.

The vendor hardware initialization emitted `InitItem` operations for operating
modes, gains, profiles, drive modes, return delay, and gripper goal current.
No separate project/user Dynamixel-register command was issued, but the
official launch itself performs these writes. This behavior must remain part
of future startup risk review.

Shutdown deactivated controllers and reported torque OFF for IDs 11-15. The
serial device had no holder and no related ROS, MoveIt, or MCP process remained.

## Real state and kinematics checkpoint

The real MCP stdio path negotiated MCP `2026-07-28` and verified descriptor,
planning-group, current-state, current-pose, FK, IK-to-current-pose, and
current-state-validity operations. The profile matched:

- manipulator `open_manipulator_x`;
- planning frame `world`;
- arm group `arm` with `joint1` through `joint4`;
- gripper group `gripper`;
- arm tool frame `end_effector_link`.

TF `world -> end_effector_link` was available. FK from the measured state and
the independently read current end-effector pose agreed closely. IK to that
same pose returned a solution close to the measured configuration, and the
measured state was valid. No IK result was executed.

## Planning-only checkpoint

The planning checkpoint took a new measurement rather than reusing a previous
state:

| Joint | Position (rad) |
|---|---:|
| `joint1` | 0.0490873852 |
| `joint2` | 0.0843689433 |
| `joint3` | 0.3712233507 |
| `joint4` | 1.1351457830 |
| `gripper_left_joint` | 0.0091658438 |

All measured velocities were zero and MoveIt reported the state valid.

### Joint-goal plan

The conservative target changed only `joint1` by `+0.02 rad`; joints 2-4
remained at their freshly measured values. The target was comfortably inside
the installed URDF limits. Velocity and acceleration scaling were both 0.1.

- target: `[0.0690873852, 0.0843689433, 0.3712233507, 1.1351457830]`;
- application plan ID: `meQLOGdNJrZKKzfifIcprxtn`;
- trajectory: 5 points, 0.399175422 seconds, joints `joint1`-`joint4`;
- retrieved plan ID and bounded metadata matched;
- validation: valid, policy `default-v1`, no findings;
- discard: successful.

### Optional pose-goal plan

After joint planning succeeded, the current measured pose was used as the
baseline. Only world-frame X was decreased by 0.005 m; Y, Z, and orientation
were unchanged.

- baseline position: `(0.1553677278, 0.0070432048, 0.0044880202)`;
- target position: `(0.1503677278, 0.0070432048, 0.0044880202)`;
- application plan ID: `5wOTseoGiMk8jlBykuZdg4A9`;
- trajectory: 8 points, 0.640165917 seconds, joints `joint1`-`joint4`;
- retrieved plan ID and bounded metadata matched;
- validation: valid, policy `default-v1`, no findings;
- discard: successful.

The optional Cartesian-path check was skipped because the successful joint and
pose planner paths provided the required planning evidence without another
target construction.

The measured positions before and after both planning operations were exactly
unchanged, and sampled velocities remained zero. Planning generated
trajectories only in memory; neither trajectory was submitted for execution.

## Execution-absence audit

The public MCP surface contained no `execute_motion_plan` Tool. This checkpoint
did not call `ExecuteTrajectory`, `FollowJointTrajectory`, plan-and-execute,
controller command topics, gripper commands, Servo/Jogging, or controller
switching. No direct physical target command was issued.

## Warnings and limitations

- FastSyncRead fell back to normal SyncRead after ten failures.
- FIFO real-time scheduling was unavailable.
- ros2_control reported URDF command-limit enforcement disabled.
- The installed gripper controller type is deprecated upstream.
- MoveIt reported absent collision geometry for `end_effector_link`, could not
  identify the SRDF end-effector parent group, and had no 3D octomap sensor.
- MoveIt used default workspace bounds and reported an unfilled planner ID.
- MoveIt required SIGTERM after not exiting within five seconds of SIGINT.
- ros2_control emitted PAL statistics context errors after successful hardware
  deactivation and torque-off.
- Physical movement cannot be assessed visually by this terminal-driven audit.
  Zero sampled velocities and unchanged measured positions demonstrate that
  planning did not command motion, but do not prove that torque engagement
  caused no brief physical response.

## Authoritative source audit

Sources consulted were the installed ROS 2 Jazzy interfaces; installed MoveIt
2.12.4 and `moveit_msgs` 2.6.0 interfaces; installed ROBOTIS
OpenMANIPULATOR-X 4.1.3 launch, URDF, SRDF, controller, kinematics, and joint
limit files; installed Dynamixel hardware interface 1.5.2 behavior and logs;
the MCP 2026-07-28 runtime surface using SDK 2.0.0; and the observed live ROS,
MoveIt, MCP, USB, and serial state.

No version discrepancy was found. The FastSyncRead incompatibility/failure and
normal-SyncRead fallback are observed runtime behavior, not an assumed API.

## Repository scope

Only this Phase 12 document was updated. No production code, configuration,
dependency, or test was changed. No hardware test suite was added. Physical
execution remains unimplemented and unverified, and no v0.2.0 work was started.
