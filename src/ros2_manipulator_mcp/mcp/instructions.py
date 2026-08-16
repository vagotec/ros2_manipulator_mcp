"""Server-level client guidance."""

SERVER_INSTRUCTIONS = (
    "Use semantic Manipulator Resources and read-only Tools before planning or "
    "policy-controlled planning-scene mutation. Planning creates reviewable "
    "server-owned plans. Physical execution exists but is disabled by default "
    "and accepts only an application-owned validated plan_id; inspect and "
    "validate before execution. Arbitrary trajectories, ROS, and controller "
    "access remain unavailable. Cancellation uses a MoveIt 2.12.4-specific "
    "compatibility mechanism. CANCELLED is not certified physical standstill."
)
