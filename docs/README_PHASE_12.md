# Phase 12 - Real OpenMANIPULATOR-X Hardware Checkpoint

## Status

- Pre-hardware audit: **COMPLETE**
- Real hardware verification: **DEFERRED**
- Reason: OpenMANIPULATOR-X hardware currently not connected
- Physical motion commands issued: **NONE**
- Torque changes performed: **NONE**
- Hardware processes started: **NONE**

Phase 12 stopped at the mandatory pre-hardware checkpoint. Real hardware
state, MCP kinematics, planning-only behavior, planning-scene behavior, and
hardware opt-in tests have not been run and are not marked as passed.

## Verified local baseline

The read-only audit verified:

| Component | Installed version |
|---|---|
| Ubuntu | 24.04.4 LTS |
| Python | 3.12.3 |
| ROS 2 | Jazzy |
| `rclpy` | 7.1.11 |
| MoveIt | 2.12.4 |
| `moveit_msgs` | 2.6.0 |
| OpenMANIPULATOR bringup | 4.1.3 |
| OpenMANIPULATOR description | 4.1.3 |
| OpenMANIPULATOR MoveIt config | 4.1.3 |
| Dynamixel hardware interface | 1.5.2 |
| Dynamixel SDK | 4.0.3 |

No `/dev/ttyUSB*`, `/dev/ttyACM*`, or `/dev/serial/by-id/*` device was
detected. Host USB enumeration showed no identifiable U2D2 or OpenCR serial
adapter. User `sarvg` is already a member of `dialout`; no permissions, udev
rules, firmware, baud rates, or hardware settings were changed.

## Verified hardware configuration

The installed official OpenMANIPULATOR-X ros2_control description selects
`dynamixel_hardware_interface/DynamixelHardware` when simulation and mock
hardware are disabled. It specifies:

- default serial device `/dev/ttyUSB0`;
- baud rate `1,000,000`;
- five Dynamixels with IDs `11` through `15`;
- position command interfaces;
- position, velocity, and effort state interfaces;
- arm joints `joint1` through `joint4`;
- gripper joint `gripper_left_joint`;
- `disable_torque_at_init=true`.

The installed controller configuration defines an arm
`JointTrajectoryController`, gripper `GripperActionController`, and
`JointStateBroadcaster`. The official MoveIt configuration maps the arm to
`FollowJointTrajectory` and the gripper to `GripperCommand`.

## Startup safety finding

The official bringup defaults `init_position` to `true`. Its installed
`joint_trajectory_executor` sends a three-second `FollowJointTrajectory` goal
for arm target `[0.0, -1.0, 1.0, 0.0]`. A future hardware verification must
therefore set:

```text
init_position:=false
```

This prevents the explicit initialization trajectory, but it does **not**
make startup guaranteed motion-free. The verified hardware path still
activates `DynamixelHardware`, enables Dynamixel torque during activation, and
activates position controllers. Torque engagement and controller activation
can produce holding response or physical movement under load.

Consequently, the current real-hardware launch path must not be started merely
for state inspection. It requires a connected and positively identified
robot, a supported safe pose, a cleared workspace, accessible power cutoff,
and fresh explicit approval acknowledging torque enablement and possible
physical response.

## Deferred future checkpoint

If Phase 12 is resumed later, the prerequisites are:

1. Connect and power the intended OpenMANIPULATOR-X interface.
2. Positively identify its stable serial device; do not assume a device name.
3. Reconcile that device with the official 1 Mbps configuration without
   changing hardware settings automatically.
4. Reconfirm the installed launch and Dynamixel lifecycle behavior.
5. Present the exact command, torque behavior, controller activation, possible
   movement, and physical precautions to the user.
6. Obtain explicit approval before starting any physical runtime.
7. Keep `init_position:=false` and never call `FollowJointTrajectory`,
   `ExecuteTrajectory`, gripper commands, direct joint commands, Dynamixel
   register writes, or controller-management operations.

The previously audited candidate command, subject to device re-verification
and explicit approval, is:

```bash
export ROS_DOMAIN_ID=67
export ROS_LOG_DIR=/tmp/ros2_manipulator_mcp_phase12_logs

ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
  use_mock_hardware:=false \
  init_position:=false \
  start_rviz:=false \
  port_name:=/dev/ttyUSB0
```

`/dev/ttyUSB0` is only the installed default, not an observed device. It must
be replaced by the positively identified device when hardware is connected.

## Authoritative source audit

The checkpoint consulted:

- installed OpenMANIPULATOR 4.1.3 launch files, ros2_control Xacro,
  controller YAML, initial-position YAML, SRDF, MoveIt controller mapping,
  kinematics, and joint-limit configuration under `/opt/ros/jazzy`;
- installed Dynamixel hardware interface 1.5.2 headers, package metadata, model
  files, and exact binary lifecycle/log strings;
- installed `joint_trajectory_executor` 4.1.3 source;
- official ROBOTIS OpenMANIPULATOR 4.1.3 source repository;
- official ROBOTIS Dynamixel hardware interface repository and Jazzy 1.5.2
  generated API documentation;
- read-only host serial-device, USB-device, user, and group inspection.

The installed launch/configuration matched the Phase 6 verified ROBOTIS 4.1.3
baseline. No live hardware/runtime comparison was possible because the robot
was not connected. Actual serial identity, detected Dynamixel models, live
joint values, timestamps, TF, ROS graph endpoints, torque state, and hardware
interoperability remain deliberately unverified.

## Files, tests, and phase boundary

Created:

- `docs/README_PHASE_12.md`

Updated:

- `docs/README_PHASES.md`

No production code, configuration, dependencies, or tests changed. No
hardware tests were created or run, and no hardware result was fabricated.
Phase 13 was not started.
