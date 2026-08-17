# ros2_manipulator_mcp Development Phases

This index records only completed or approved project phases. Later phases are not implied to be implemented.

| Phase | Documentation | Status |
|---|---|---|
| 0 - Architecture Analysis | Phase 0 review | Complete; analysis only |
| 1 - Project Foundation | [README_PHASE_1.md](README_PHASE_1.md) | Complete |
| 2 - Manipulator Domain | [README_PHASE_2.md](README_PHASE_2.md) | Complete |
| 3 - Application Layer and Backend Ports | [README_PHASE_3.md](README_PHASE_3.md) | Complete |
| 4 - Safety and Policy Layer | [README_PHASE_4.md](README_PHASE_4.md) | Complete |
| 5 - ROS 2 Jazzy / MoveIt 2 Adapter Foundation | [README_PHASE_5.md](README_PHASE_5.md) | Complete |
| 6 - Live MoveIt Integration and OpenMANIPULATOR-X Baseline | [README_PHASE_6.md](README_PHASE_6.md) | Complete |
| 7 - MCP Tools for v0.1.0 | [README_PHASE_7.md](README_PHASE_7.md) | Complete |
| 8 - MCP Resources and Health/Safety Context | [README_PHASE_8.md](README_PHASE_8.md) | Complete |
| 9 - MCP Prompts for v0.1.0 | [README_PHASE_9.md](README_PHASE_9.md) | Complete |
| 10 - Server Composition and MCP End-to-End | [README_PHASE_10.md](README_PHASE_10.md) | Complete |
| 11 - OpenMANIPULATOR-X Full MCP Simulation E2E | [README_PHASE_11.md](README_PHASE_11.md) | Complete |
| 12 - Real OpenMANIPULATOR-X Hardware Verification | [README_PHASE_12.md](README_PHASE_12.md) | Read/kinematics and planning-only verification complete; physical execution not performed |
| 13 - v0.1.0 Final Audit and Release Preparation | [README_PHASE_13.md](README_PHASE_13.md) | Complete; ready with documented hardware-verification limitation |
| 14 - v0.2.0 Execution Architecture and Safety Design | [README_PHASE_14.md](README_PHASE_14.md) | Complete; analysis and design only |
| 15 - Execution Domain, State Machine, and Plan Reservation | [README_PHASE_15.md](README_PHASE_15.md) | Complete; backend-neutral foundation only |
| 16 - Pre-execution Revalidation and Safety Gate | [README_PHASE_16.md](README_PHASE_16.md) | Complete; no execution adapter or motion |
| 17 - Production Execution Backend and Workflow | [README_PHASE_17.md](README_PHASE_17.md) | Complete; mock-hardware verification only |
| 18 - MCP Execution Surface and Mock E2E | [README_PHASE_18.md](README_PHASE_18.md) | Complete; physical execution remains disabled by default |
| 19 - v0.2.0 Mock Execution Audit and RC Preparation | [README_PHASE_19.md](README_PHASE_19.md) | Complete; mock-only robustness audit |
| 20 - v0.2.0 Execution Verification | [README_PHASE_20.md](README_PHASE_20.md) | Complete for v0.2.0; real execution verified, mock cancellation verified, accepted hardware limitations documented |
| 21 - v0.2.0 Final Audit and Release Readiness | [README_PHASE_21.md](README_PHASE_21.md) | Complete; release-ready with documented limitations |

The approved architecture is:

```text
MCP
 -> Application
 -> Manipulator Domain
 -> Safety / Validation
 -> Manipulator Adapter Port
 -> ROS 2 Jazzy / MoveIt 2 Adapter
 -> MoveIt 2 / ROS 2
```

MoveIt 2 remains replaceable backend infrastructure. `ros2_control_mcp` separately owns controllers, hardware lifecycle, resource claims, and controller switching.

## Authoritative source rule

Before implementing version-sensitive external APIs, every phase must verify the exact interface against primary sources in this order: official specification, version-specific documentation, upstream source/interface definitions, then official examples/tests. The baselines are Ubuntu 24.04, Python 3.12+, ROS 2 Jazzy, MCP specification `2026-07-28`, MCP Python SDK 2.0.0, and the official Jazzy MoveIt 2 and ros2_control interfaces.

Installed versions take precedence when they differ from documentation, and the discrepancy must be reported. ROS 1, MoveIt 1, other ROS distributions, blogs, Q&A sites, and the two sibling MCP repositories are not authoritative API sources. Each phase report must include an authoritative-source audit; an unverifiable interface remains an open issue rather than being invented.
