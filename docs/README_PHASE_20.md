# Phase 20 - v0.2.0 Real-Hardware Checkpoints

## Phase 20A state preflight

Phase 20A completed the repeated real-hardware state preflight with classification
`A - REPEATABLE VALID STATE`. The verified OpenMANIPULATOR-X used `/dev/ttyUSB0`,
the stable FTDI by-id link, 1,000,000 baud, and Dynamixel IDs 11-15. The final
preflight state was valid and stationary, TF `world -> end_effector_link` was
available, and joint4 was stable near `1.4342720367 rad` with raw position 2983
before, during, and after bringup. No trajectory was issued in Phase 20A. The
runtime was shut down with Torque OFF for all five IDs and the serial port was
released.

## Phase 20B first controlled physical execution

On 2026-08-16 the operator explicitly confirmed physical observation, mechanical
stability, a clear workspace and motion envelope, cable and end-effector
clearance, and immediate access to physical power cutoff. Phase 20B then used
exactly the audited real-hardware launch in ROS domain 67 with
`use_mock_hardware:=false`, `init_position:=false`, `start_rviz:=false`, and
`port_name:=/dev/ttyUSB0`. The actual MCP stdio path performed one and only one
call to `execute_motion_plan`.

The fresh pre-plan arm state was:

| Joint | Position (rad) | Velocity (rad/s) |
|---|---:|---:|
| joint1 | -0.01073786551540623 | 0.0 |
| joint2 | 0.5491651220628526 | 0.0 |
| joint3 | -0.3344078117592768 | 0.0 |
| joint4 | 1.4005244593393833 | 0.0 |

The MoveIt state-validity result was true. The scene was empty and had revision
`e8d037bb06e531ca32ed11cea9dc5b302589e8d08b2b24f6a2c2ddbce31b8fec`.
The only intended target changed joint1 by `+0.01 rad`, to
`-0.0007378655154062306 rad`; joint2 through joint4 were copied exactly from the
fresh state, and no gripper target was included. Both velocity and acceleration
scaling were `0.05` under the dedicated temporary policy
`phase20b-real-single-motion`.

Planning produced plan ID `gcNyktN-gb6iMGCWQEZLpumq`, five trajectory points,
and duration `0.399652787 s`; MoveIt planning took `0.02120186200045282 s`.
Stored-plan validation returned valid with no findings and the matching policy
identity. Production execution revalidation enforced execution enablement,
application ownership and reservation, policy/group identity, state freshness,
the `0.02 rad` start tolerance, exact scene revision, single-active-execution,
and backend availability before adapter submission.

Execution ID `_Hel-PSK5AbjfcO-2reeg6lK` progressed
`VALIDATING -> RUNNING -> SUCCEEDED`. MoveIt reported that the arm controller
accepted the goal, reached it successfully, and completed execution with status
`SUCCEEDED`. No stop or cancellation was requested, no failure was recorded,
and the backend was not quarantined. Acceptance permanently consumed the plan
through the application registry; the plan was not executed a second time merely
to demonstrate rejection.

After 0.75 seconds the measured state was valid and all measured velocities were
zero:

| Joint | Planned endpoint (rad) | Measured endpoint (rad) | Absolute error (rad) | Measured delta (rad) |
|---|---:|---:|---:|---:|
| joint1 | -0.0007378655154062306 | -0.004601942363863909 | 0.0038640768484576784 | +0.006135923151542322 |
| joint2 | 0.5491651220628526 | 0.5476311412749668 | 0.0015339807878858025 | -0.0015339807878858025 |
| joint3 | -0.3344078117592768 | -0.3344078117592768 | 0.0 | 0.0 |
| joint4 | 1.4005244593393833 | 1.4005244593393833 | 0.0 | 0.0 |

These are command/result and measured joint-state data from one conservative
test, not visual confirmation or a certified positioning or machinery-safety
claim. The operator explicitly reported that no physical movement was visibly
observed. Therefore the `SUCCEEDED` result and approximately `+0.00614 rad`
measured joint1 displacement do not independently confirm visible physical
motion. joint2's small measured change is recorded as ordinary control/readback
noise for this checkpoint; it was not commanded as a changed target. The gripper
position remained `0.011210531884359998 rad`.

## Phase 20B.2 second controlled execution

The operator provided a new seven-condition physical-safety confirmation. From
the fresh state, the MCP request changed only joint1 by `+0.02 rad`; joint2
through joint4 were copied unchanged and no gripper target was sent. Velocity
and acceleration scaling were both `0.05` under temporary policy
`phase20b2-real-single-motion`.

The fresh arm state was joint1 `-0.013805827091177836`, joint2
`0.3390097541225199`, joint3 `0.004601942363450018`, and joint4
`1.2363885150356202 rad`, with all velocities zero. State validity was true and
the scene revision was
`e8d037bb06e531ca32ed11cea9dc5b302589e8d08b2b24f6a2c2ddbce31b8fec`.
Plan `cMo5hXuQqHOaPLZhPrqCSbCw` contained seven points and lasted
`0.564690159 s`. Its first point matched the fresh state. The final MoveIt point
was `[0.006123858910650105, 0.3389966556945406, 0.004625234291817707,
1.236289993292371]`. Immediately before execution, state age was
`0.0037183762 s`, maximum start deviation was `0.0015339808 rad`, scene revision
matched exactly, and stored-plan validation returned valid with no findings.

Execution `atxZeim91_Plvy3t2UzFJP3h` progressed
`VALIDATING -> STARTING -> RUNNING -> SUCCEEDED`. Read-only capture collected 142
joint-state samples. The measured joint1 displacement was
`+0.01380582709097089 rad`, with maximum sampled velocity
`0.0479382454 rad/s`. The requested joint1 target was
`0.006194172908822165 rad`; final measured joint1 was approximately zero, giving
absolute target error `0.00619417290902911 rad`. joint2 and joint3 each changed
`-0.0015339807878858025 rad`; joint4 was unchanged. All final velocities were
zero and post-state validity was true.

The operator reported that no physical movement was visibly observed. Direction
and requested magnitude therefore were not visually confirmed. This remains
separate from both the `SUCCEEDED` result and telemetric displacement.

## Phase 20B.3 third controlled execution

The operator again provided a new seven-condition safety confirmation. The fresh
MCP state was joint1 approximately zero (`-2.0694557179012918e-13`), joint2
`0.3390097541225199`, joint3 `0.0030679615755642153`, and joint4
`1.2348545342477344 rad`, with all velocities zero. The state was valid and all
arm joints were inside the installed URDF limits.

The only intended change was joint1 `+0.03 rad`, producing target
`0.029999999999793053 rad`; joint2 through joint4 were copied unchanged, and no
gripper target was sent. Velocity and acceleration scaling remained `0.05` under
temporary policy `phase20b3-real-single-motion`.

Plan `tNqGURFqBARR34Q3LyCpNvAR` contained eight points and lasted
`0.692910511 s`. Its first point matched the fresh start exactly. Its final
MoveIt point was `[0.030007811000736236, 0.3391009436123129,
0.0031480798438733764, 1.234841129414209]`; the sub-`0.0001 rad` differences in
joint2 through joint4 were recorded before execution. Immediate state age was
`0.0038270950 s`, all start deviations were zero, scene revision matched
exactly, and stored-plan validation returned valid with no findings.

Execution `YknvMwW5Ssp6-3mpu9yWjK_k` progressed
`VALIDATING -> RUNNING -> SUCCEEDED`. Read-only capture collected 151 samples.
The measured joint1 displacement was `+0.023009711818284817 rad`, with maximum
sampled velocity `0.0479382454 rad/s`. Final measured joint1 was
`0.02300971181807787 rad`, giving target error `0.006990288181715182 rad`.
joint2 and joint3 were unchanged; joint4 changed
`-0.0015339807878858025 rad`. All final velocities were zero and post-state
validity was true. The accepted plan was consumed and no quarantine was entered.

The operator separately reported:

- no joint1 movement was visible;
- direction and `+0.03 rad` magnitude could not be visually confirmed;
- no unexpected movement occurred;
- no abnormal sound, vibration, or oscillation occurred;
- no cable or end-effector issue occurred.

Phase 20B.3 therefore records the evidence categories independently:

| Evidence | Result |
|---|---|
| MCP / MoveIt execution | `SUCCEEDED` |
| Joint-state telemetry | joint1 `+0.0230097118 rad` |
| Operator-visible physical movement | Not observed |

No `cancel_execution` call, automatic retry, additional trajectory, direct
controller command, or gripper command occurred. Phase 20C was not started.

## Runtime observations and shutdown

All IDs initialized and torque was enabled. Initial FastSyncRead failed its
bounded retries and the hardware interface switched successfully to normal
SyncRead; no continuing communication or Dynamixel hardware error followed.
MoveIt emitted the already known missing end-effector collision geometry,
end-effector parent-group, workspace-default, and absent Octomap sensor warnings.
The controller manager also reported unavailable FIFO real-time scheduling and
the installed deprecated gripper-controller type. None prevented the approved
arm trajectory.

The MCP process closed cleanly after readback. MoveIt did not exit within five
seconds of SIGINT and the ROS launch supervisor applied the previously observed
bounded SIGTERM escalation. Hardware shutdown deactivated all controllers and
`OpenManipulatorXSystem`; IDs 11, 12, 13, 14, and 15 each reported Torque OFF.
`/dev/ttyUSB0` was released and no related ROS, MoveIt, or MCP process remained.
The controller-manager statistics threads emitted their known invalid-context
shutdown messages only after successful hardware deactivation.

No direct `FollowJointTrajectory` call was made by MCP or project code; MoveIt
internally selected the configured arm controller behind `ExecuteTrajectory`.
`cancel_execution` was not called, physical cancellation remains untested, and
Phase 20C was not started.

## Authoritative-source audit

The checkpoint rechecked the installed OpenMANIPULATOR 4.1.3 bringup launch,
URDF/xacro, ros2_control hardware definition, joint limits, and MoveIt controller
configuration under `/opt/ros/jazzy/share`. They verify `/dev/ttyUSB0`,
1,000,000 baud, IDs 11-15, joint1 limits `[-pi, pi]`, the four-joint `arm` group,
and MoveIt's `arm_controller/follow_joint_trajectory` backend. Runtime discovery
verified `/execute_trajectory` as `moveit_msgs/action/ExecuteTrajectory`, the
configured controller actions, active controllers, joint-state topic, and the
installed ROS 2 Jazzy graph before execution.

The exact baseline remained ROS 2 Jazzy, rclpy 7.1.11, MoveIt 2.12.4,
moveit_msgs 2.6.0, ros2_control 4.45.2, joint_trajectory_controller 4.40.1,
OpenMANIPULATOR 4.1.3, dynamixel_hardware_interface 1.5.2, MCP specification
2026-07-28, MCP Python SDK 2.0.0, and mcp-types 2.0.0. No interface discrepancy
was found. The test-specific 0.05 scaling values and observation tolerances are
project policy, not ROS or MoveIt standards.

## Phase 20C.1 complete-actuator attempt (2026-08-17)

The operator supplied one fresh twelve-condition safety confirmation for a
single sequential `joint1 -> joint2 -> joint3 -> joint4 -> gripper` run. The
real hardware started with `/dev/ttyUSB0`, 1,000,000 baud, IDs 11-15 and
`init_position=false`. All five IDs answered ping, Torque ON was reported for
all five, and the arm, gripper and joint-state controllers became active. The
known FastSyncRead `-3001` startup failure exhausted ten attempts, then changed
successfully to normal SyncRead. Communication and controller operation
continued after that fallback.

MoveIt and the production MCP path started successfully. Each attempted arm
motion used a fresh state, state-validity and candidate-validity checks,
collision-aware planning, one changed arm target, and velocity and acceleration
scaling of `0.05`. No direct controller call, cancellation or retry was used.

The sequence stopped after the third actuator because the fresh post-execution
sample for joint3 still had absolute velocity above the run's `0.01 rad/s`
quiet threshold after the 0.75-second settling interval. This was treated as a
material stop condition. The client exception occurred before its aggregate
JSON report was emitted, so exact positions, plan IDs and measured deltas from
this attempt are not available as durable evidence and must not be inferred
from controller success.

| Actuator | MCP / MoveIt / controller evidence | Telemetry evidence from this attempt | Operator-visible evidence |
|---|---|---|---|
| joint1 / ID 11 | One goal accepted; controller and MoveIt reported `SUCCEEDED` | Post-check completed with velocity within `0.01 rad/s`; numeric state was not durably emitted | Awaiting consolidated operator report |
| joint2 / ID 12 | One goal accepted; controller and MoveIt reported `SUCCEEDED` | Post-check completed with velocity within `0.01 rad/s`; numeric state was not durably emitted | Awaiting consolidated operator report |
| joint3 / ID 13 | One goal accepted; controller and MoveIt reported `SUCCEEDED` | Post-check found absolute joint3 velocity above `0.01 rad/s`; numeric state was not durably emitted | Awaiting consolidated operator report |
| joint4 / ID 14 | Not planned or executed after the stop condition | No new telemetry verification | Not tested in this attempt |
| gripper / ID 15 | Not planned or executed after the stop condition | No new telemetry verification | Not tested in this attempt |

The stop path closed MCP, terminated MoveIt, deactivated all controllers and
`OpenManipulatorXSystem`, and reported Torque OFF for IDs 11-15. The serial
device was released and no relevant runtime remained. No Phase 20C.2 or Phase
21 operation occurred. This attempt did not complete the five-actuator matrix.

## Phase 20C.1 forward-and-return attempt (2026-08-17)

After another fresh twelve-condition operator safety confirmation, the audited
real runtime was started again. IDs 11-15 answered ping, the known FastSyncRead
`-3001` sequence fell back successfully to stable normal SyncRead, Torque ON was
reported for all IDs, and all controllers and MoveIt became ready. The MCP
health resource reported `ready`, physical execution available, and the
corrected gripper descriptor exposed only `gripper_left_joint` as commandable.

Eight arm-controller goals were accepted and completed successfully: outward
and return for each of joints 1 through 4. Every requested arm displacement was
at most `0.174532925 rad` (approximately 10 degrees), used velocity and
acceleration scaling `0.05`, and changed only the tested joint intentionally.
Fresh validity, limits and collision checks preceded planning. Bounded
post-result sampling required the tested joint velocity to remain within
`0.01 rad/s` for 0.5 seconds; reported final velocities were zero.

| Actuator | Outward MCP/controller result | Outward telemetry | Return MCP/controller result | Return telemetry |
|---|---|---|---|---|
| joint1 / ID 11 | `SUCCEEDED`; +10-degree target from `-0.007669904 rad` | Endpoint around `0.164-0.166 rad`; exact aggregate record was not emitted before a diagnostic-only sampling exception | `SUCCEEDED`; plan `-qP4g1Yd9AqHnbz-0JibpyJ3`, execution `Xiw9DR8wNNiaX8X6jTTRKCa3` | `-0.004601942 rad`, return delta `-0.170271867 rad` (`-9.755859375 deg`), original error `+0.003067962 rad`, velocity `0` |
| joint2 / ID 12 | `SUCCEEDED`; plan `7Mk9Gyd6AWV187f0spvBz1Vk`, execution `fgXU2vLgBijXqLspBos-3bna` | `0.474000063 -> 0.312932081 rad`, delta `-0.161067983 rad` (`-9.228515625 deg`), velocity `0` | `SUCCEEDED`; exact aggregate record was not emitted before a diagnostic-only sampling exception | Endpoint around `0.464796179 rad`, original error about `-0.009203885 rad`, sampled velocity `0` |
| joint3 / ID 13 | `SUCCEEDED`; plan `DEcqic-0iRqvHz1XEcvf8OP7`, execution `L9j7lmLGCuG_7j1NAgI4C3LL` | `-0.434116563 -> -0.599786488 rad`, delta `-0.165669925 rad` (`-9.4921875 deg`), velocity `0` | `SUCCEEDED`; plan `uYjMUlcRt-2FZuydjrM4eT3y`, execution `aIPecjcnfYBPRzzqAsHAZl_N` | `-0.601320469 -> -0.447922390 rad`, delta `+0.153398079 rad` (`+8.7890625 deg`), original error `-0.013805827 rad`, velocity `0` |
| joint4 / ID 14 | `SUCCEEDED`; plan `BJjcSZb09Wtlxh4wQad_LSG5`, execution `Ao-Qii47BiMLDht4FWOabv_I` | `1.567728365 -> 1.741068194 rad`, delta `+0.173339829 rad` (`+9.931640625 deg`), target error `+0.000340885 rad`, velocity `0` | `SUCCEEDED`; plan `x98D9LCAjFCq9NNkyi65u1la`, execution `0BbdIdI0yACE7KF3YIzimOhU` | `1.741068194 -> 1.567728365 rad`, delta `-0.173339829 rad` (`-9.931640625 deg`), original error `+0.001533981 rad`, velocity `0` |
| gripper / ID 15 | Not executed | Fresh measured position `0.0203764 m` exceeded the authoritative `0.0200000 m` upper bound | Not executed | No command was submitted |

The gripper planning request failed with MoveIt error `99999` because
`CheckStartStateBounds` found both the commandable left joint and its right
mimic state at `0.0203764 m`, outside the installed `[-0.011, 0.020] m` range.
MoveIt aborted the planning pipeline before producing a plan, so no gripper
execution or retry occurred. This is a real state/limit discrepancy requiring
separate diagnosis; the application correctly preserved the safety boundary.

The runtime then shut down: MCP and MoveIt stopped, controllers and
`OpenManipulatorXSystem` deactivated, IDs 11-15 each reported Torque OFF,
`/dev/ttyUSB0` was released, and no relevant process remained. Operator-visible
evidence is intentionally pending and is not inferred from the eight successful
arm-controller results or joint-state telemetry. No cancellation, Phase 20C.2,
Phase 21 or commit occurred.

## Phase 20C.1 ID 15 valid-range function test (2026-08-17)

With actuator power on and Torque OFF, a direct read-only Protocol 2.0 check at
1,000,000 baud measured ID 15 at raw `1519`, which the installed production
conversion maps to `-0.0093244825 m`. Present Velocity and Hardware Error Status
were both zero. The value was inside the authoritative URDF range
`[-0.011, +0.020] m`; the diagnostic made no register write.

The audited real hardware, MoveIt and MCP runtime then created and validated a
production-path gripper plan. Plan `BE3Jf8VGa7cZ5MXa5NRe2Ymx` commanded only
`gripper_left_joint` from `-0.0093244825 m` toward OPEN at
`-0.0073244825 m`, a requested `+2.0000000 mm` displacement. Its three-point
trajectory duration was `0.175604141 s`, velocity and acceleration scaling were
both `0.05`, and validation returned valid with no findings.

After separate operator approval, execution
`ixlu0CmDiZ9JAbVuitFqLBpq` progressed `validating -> running -> succeeded`.
The controller accepted the goal. Fresh telemetry measured an endpoint of
`-0.0074031807 m`, an actual `+1.9213018 mm` displacement, target error
`-0.0786982 mm`, and final velocity zero. No hardware or controller error was
reported. Separately, the operator confirmed that ID 15/gripper movement was
physically visible, described as a very small, short "tick" rather than a large
opening or closing movement. Thus the evidence remains distinct:

- MCP/MoveIt/controller: `SUCCEEDED`;
- telemetry: `+1.9213018 mm`, final velocity zero;
- operator observation: visible physical motion VERIFIED, but very small.

No arm joint was intentionally commanded, and there was no retry, cancellation,
direct Dynamixel Goal Position command, Phase 20C.2, Phase 21, or commit.

## Phase 20C.1 final v0.2.0 classification

**COMPLETE FOR v0.2.0 WITH DOCUMENTED GRIPPER CALIBRATION LIMITATION.**

The accumulated checkpoints verified the complete production lifecycle on a
real OpenMANIPULATOR-X: the FTDI U2D2 at `/dev/ttyUSB0`, Dynamixel IDs 11-15,
the real `dynamixel_hardware_interface/DynamixelHardware` plugin, controller
startup, Torque ON, fresh joint-state feedback, MoveIt, MCP planning and
execution, controller and hardware deactivation, Torque OFF for IDs 11-15, and
release of the serial device. No hardware runtime remained active at this final
documentation checkpoint.

The final actuator matrix deliberately keeps controller result, measured state,
and human observation separate:

| ID / actuator | MCP / MoveIt / controller evidence | Telemetry / hardware-state evidence | Operator-visible physical evidence | v0.2.0 classification |
|---|---|---|---|---|
| 11 / joint1 | Real plans executed through the production path and returned `SUCCEEDED` | Real joint-state displacement and settled final velocity were measured | Visible joint1 motion confirmed | Verified |
| 12 / joint2 | Real plans executed through the production path and returned `SUCCEEDED` | Real joint-state displacement and settled final velocity were measured | Visible joint2 motion observed during the later complete actuator sequence | Verified; historical shutdown cause unresolved |
| 13 / joint3 | Real outward and return plans returned `SUCCEEDED` | Outward `-0.165669925 rad`; return `+0.153398079 rad`; reported final velocities zero | Visible joint3 motion observed during the later complete actuator sequence | Verified |
| 14 / joint4 | Real outward and return plans returned `SUCCEEDED` | Outward and return each measured `0.173339829 rad` in opposite directions; reported final velocities zero | Visible joint4 motion observed during the later complete actuator sequence | Verified |
| 15 / gripper | MCP -> application -> MoveIt -> `gripper_controller` -> ID 15 returned `SUCCEEDED` | Requested `+2.0000 mm`, measured `+1.9213018 mm`, final velocity zero; subsequent CLOSE and RETURN also succeeded | Visible physical gripper/ID 15 movement confirmed, described as a small tick | Motor/control path verified; calibration limitation remains |

### ID 15 CLOSE and RETURN evidence

The later authorized two-leg sequence changed only `gripper_left_joint`.
CLOSE plan `OmRbcasH1YCGnz_N0sbFGtfy`, execution
`Pl9Rqafs5kRwH3T3SAXwzjRB`, progressed
`validating -> running -> succeeded`: start `-0.0074031807 m`, requested target
`-0.0100000000 m`, measured endpoint `-0.0099237876 m`, measured displacement
`-2.5206069 mm`, target error `+0.0762124 mm`, and final velocity zero. RETURN
plan `FtCVzvaE_YRX6T5bW1cTgMsH`, execution
`Ck_CYwt8WfRJytG7V2XbVI31`, also progressed
`validating -> running -> succeeded`: start `-0.0099237876 m`, requested target
`-0.0074031807 m`, measured endpoint `-0.0073503009 m`, measured displacement
`+2.5734868 mm`, return error `+0.0528798 mm`, and final velocity zero. Both
plans validated with no findings and no hardware or controller error was
reported. These controller and telemetry results are not used as substitutes
for operator-visible evidence.

### Known real-hardware calibration / mechanical reference limitation

ID 15 communication, execution, telemetry and visible motor/mechanism movement
are verified. Separately, manual Torque-OFF endpoint observations found physical
FULLY OPEN around raw `1613-1713` and physical FULLY CLOSED around raw `427`.
Those values do not correspond to the installed official OpenMANIPULATOR-X
4.1.3 conversion and SRDF/URDF reference positions. This is classified as a
**KNOWN REAL-HARDWARE CALIBRATION / MECHANICAL REFERENCE LIMITATION**, not as a
demonstrated MCP, MoveIt, controller, or ID 15 communication failure.

Mechanical inspection, re-indexing, or calibration is deferred beyond v0.2.0.
The URDF, SRDF, gripper conversion, Dynamixel Homing Offset, EEPROM limits, and
production safety limits remain unchanged.

### Historical ID 12 event

One earlier ID 12 shutdown/error event with continuous LED blinking occurred.
After a physical power cycle, a read-only diagnostic found Hardware Error Status
`0x00`, Torque OFF, `12.1 V`, `34 C`, and zero velocity. ID 12 subsequently
communicated and executed real movement successfully, with no recurring
continuous blinking during the later successful tests. Because the latched
status was cleared by the physical power cycle before it could be preserved,
the initiating cause remains unknown and must not be presented as diagnosed.

### FastSyncRead fallback

The tested hardware repeatedly produced FastSyncRead error `-3001` during
startup. The official driver exhausted its bounded FastSyncRead attempts and
fell back to normal SyncRead, after which communication and real execution
operated successfully. This known successful fallback alone is not classified
as a hardware failure; a persistent failure after fallback would remain a stop
condition.

### Gripper mimic-joint production correction

The OpenMANIPULATOR-X profile now exposes only the independently commandable
`gripper_left_joint` in the gripper planning-group descriptor while retaining
the right finger link; `gripper_right_joint` remains a URDF mimic joint. This
corrects the execution-validation defect that previously treated the mimic
joint as independently required. Focused profile and pre-execution regression
tests cover the corrected invariant. The implementation was not redesigned
during finalization.

No physical cancellation test was performed during Phase 20C.1. At its
finalization checkpoint, Phase 20C.2 and Phase 21 had not been started, and no
commit, push, merge, tag, or release had been created.

## Phase 20C.2 cancellation classification for v0.2.0

**COMPLETE FOR v0.2.0 — MOCK VERIFIED / REAL-HARDWARE CANCELLATION
DEFERRED.**

No additional cancellation experiment was required. The classification uses
the existing official OpenMANIPULATOR-X mock-hardware evidence from Phases 17
through 19. That evidence verified normal ExecuteTrajectory execution, an
active execution, the project `cancel_execution` path, the Jazzy/MoveIt 2.12.4
compatibility stop event, TrajectoryExecutionManager stop processing,
controller cancellation, a causally related MoveIt `PREEMPTED` result,
measured-state stabilization, and the final project state `CANCELLED`. It also
covered exactly-once stop dispatch, repeated/idempotent cancellation,
successful execution after confirmed cancellation, timeout handling, backend
quarantine for ambiguous or unsafe outcomes, and shutdown with an accepted
active execution.

Phase 17A/17B established on the exact MoveIt 2.12.4 baseline that ordinary
public ExecuteTrajectory cancellation through `cancel_goal_async()` is not
reliably processed while execution is active. The production adapter therefore
uses the audited version-specific compatibility path: publish one
`std_msgs/msg/String` value `stop` on `/trajectory_execution_event`, which
enters MoveIt's TrajectoryExecutionManager stop path. The resulting controller
cancellation and MoveIt `PREEMPTED` result are treated as causal evidence only
when they follow the project-owned stop request.

The release classification is:

| Cancellation target | v0.2.0 classification | Evidence boundary |
|---|---|---|
| Official OpenMANIPULATOR-X mock hardware | **VERIFIED** | Full cancel path, causal terminal evidence, stabilization, idempotence, reuse, timeout/quarantine and active-shutdown behavior verified |
| Real OpenMANIPULATOR-X hardware | **NOT PHYSICALLY RELEASE-VERIFIED** | Deliberately deferred; no physical cancellation trajectory was executed |

Real-hardware cancellation is an accepted v0.2.0 limitation. Cancellation is
more safety-sensitive than normal execution, and the MoveIt compatibility stop
mechanism is not machinery-safety functionality. A project `CANCELLED` state
requires the causal `PREEMPTED` and bounded measured-state stabilization
evidence defined by the application, but it does not certify physical
standstill. The physical power cutoff remains the emergency mechanism and must
be immediately available during physical operation.

The shipped configuration remains fail-closed with
`execution.enabled = false`; physical execution requires an explicit opt-in.
`cancel_execution` is neither an emergency stop nor certified machinery-safety
functionality. The `/trajectory_execution_event` compatibility mechanism is
specific to the audited Jazzy/MoveIt 2.12.4 implementation. Every newly
supported MoveIt version must reverify its stop dispatch, causal result,
controller behavior, stabilization, timeout and quarantine semantics before
physical execution support can be claimed.

Phase 20C.2 changed documentation only. It did not use hardware, start ROS,
MoveIt, ros2_control or MCP, communicate with Dynamixels, enable torque, execute
a trajectory, or rerun the already successful mock cancellation integration.
Phase 21 was not started, and no commit, push, merge, tag, or release was
created.

## Phase 20 series closure

**PHASE 20 SERIES: COMPLETE FOR v0.2.0.**

The final closure audit confirmed the accumulated Phase 20 evidence and current
production boundaries:

- Real OpenMANIPULATOR-X startup detected IDs 11-15 through the
  `dynamixel_hardware_interface/DynamixelHardware` plugin.
- The lifecycle was verified from startup through Torque ON, active controller
  operation, orderly deactivation, Torque OFF for IDs 11-15, and serial-device
  release.
- IDs 11, 12, 13 and 14 each have real production-path execution, measured
  joint-state movement and separate operator-confirmed visible movement.
- ID 15 has verified communication, production-path execution, measured
  movement and separate operator-confirmed visible physical movement.
- The verified real execution chain was
  `MCP -> application -> safety/execution validation -> Jazzy adapter -> MoveIt
  ExecuteTrajectory -> ros2_control -> real Dynamixel hardware`.
- Application-owned immutable plans retain reservation, fresh-state and exact
  scene revalidation, validation, backend acceptance and one-way consumption
  semantics; consumed plans cannot be executed again.
- The shipped default remains `execution.enabled = false`, so physical
  execution is explicitly opt-in.
- The public MCP surface exposes bounded manipulator-specific inspection,
  planning, scene, stored-plan, execution-status and cancellation operations.
  It exposes no arbitrary ROS service, action or topic access and no arbitrary
  shell execution.
- There is no public direct `FollowJointTrajectory` interface. Production
  execution accepts only an application-owned plan ID and submits its validated
  trajectory through MoveIt ExecuteTrajectory; controller selection remains
  internal to MoveIt.
- The gripper profile exposes only independently commandable
  `gripper_left_joint`; `gripper_right_joint` remains a URDF mimic joint and is
  not independently required by the execution gate.
- Phase 20 documentation preserves separate MCP/MoveIt/controller,
  telemetry/hardware-state and operator-visible evidence classes.

The accepted non-blocking v0.2.0 limitations are:

1. real-hardware cancellation is not physically release-verified; mock-hardware
   cancellation is verified;
2. the tested ID 15 mechanism has a mechanical/encoder-reference mismatch
   relative to the official model, with re-indexing/calibration deferred;
3. the initiating cause of the historical ID 12 shutdown remains unknown,
   although later execution succeeded without recurring continuous blinking;
4. FastSyncRead may produce `-3001` at startup before the official driver
   successfully falls back to normal SyncRead.

`cancel_execution` is not an emergency stop or certified machinery-safety
functionality. Project `CANCELLED` requires causal MoveIt `PREEMPTED` and
measured-state stabilization evidence but does not certify physical standstill.
A physical power cutoff remains required for emergency handling.

This closure changed documentation only. It did not start ROS, MoveIt,
ros2_control, MCP or Dynamixel communication, enable torque, or use physical
hardware. Phase 21 was not started, and no commit, push, merge, tag or release
was created.
