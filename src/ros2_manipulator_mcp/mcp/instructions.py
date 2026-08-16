"""Server-level client guidance."""

SERVER_INSTRUCTIONS = (
    "Use semantic Manipulator Resources and read-only Tools before planning or "
    "policy-controlled planning-scene mutation. Planning creates reviewable "
    "server-owned plans and is separate from physical execution. Physical "
    "execution and arbitrary ROS access are unavailable in v0.1.0. Validation "
    "is policy context, not certified physical safety."
)
