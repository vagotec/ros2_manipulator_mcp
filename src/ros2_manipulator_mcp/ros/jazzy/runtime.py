"""Managed rclpy service and topic runtime for the Jazzy adapter."""

import asyncio
from threading import Event, Lock, Thread
from typing import Any, Protocol

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


class ServiceUnavailableError(RuntimeError):
    """A configured ROS service did not become available in time."""


class RosServiceRuntime(Protocol):
    """Internal testable boundary around concrete rclpy resources."""

    async def call_service(
        self,
        service_type: type,
        service_name: str,
        request: Any,
        timeout_seconds: float,
    ) -> Any:
        """Call one fixed typed service with a bounded timeout."""
        ...

    async def read_message(
        self,
        message_type: type,
        topic_name: str,
        timeout_seconds: float,
    ) -> Any:
        """Read one typed topic message with a bounded timeout."""
        ...

    def now_seconds(self) -> float:
        """Return current time in the node clock's domain."""
        ...

    async def send_action_goal(
        self,
        action_type: type,
        action_name: str,
        goal: Any,
        server_timeout_seconds: float,
        acceptance_timeout_seconds: float,
    ) -> Any:
        """Submit one typed action goal with bounded readiness and acceptance."""
        ...

    async def wait_action_result(self, goal_handle: Any, timeout_seconds: float) -> Any:
        """Wait boundedly for one accepted action result."""
        ...

    def publish_message(self, message_type: type, topic_name: str, message: Any) -> None:
        """Publish one message to a fixed adapter-owned topic."""
        ...

    def close(self) -> None:
        """Release all ROS resources."""
        ...


class JazzyRosRuntime:
    """Own an isolated context, node, executor, clients, and spin thread."""

    def __init__(self, node_name: str) -> None:
        self._context = Context()
        self._clients: dict[tuple[type, str], Any] = {}
        self._action_clients: dict[tuple[type, str], Any] = {}
        self._publishers: dict[tuple[type, str], Any] = {}
        self._client_lock = Lock()
        self._closed = False
        try:
            rclpy.init(
                context=self._context,
                signal_handler_options=SignalHandlerOptions.NO,
            )
            self._node = Node(node_name, context=self._context)
            self._executor = MultiThreadedExecutor(
                num_threads=2,
                context=self._context,
            )
            self._executor.add_node(self._node)
            self._thread = Thread(
                target=self._executor.spin,
                name=f"{node_name}-executor",
                daemon=True,
            )
            self._thread.start()
        except Exception:
            if self._context.ok():
                self._context.shutdown()
            raise

    async def call_service(
        self,
        service_type: type,
        service_name: str,
        request: Any,
        timeout_seconds: float,
    ) -> Any:
        """Call a service outside the event-loop thread."""
        self._ensure_open()
        return await asyncio.to_thread(
            self._call_service_blocking,
            service_type,
            service_name,
            request,
            timeout_seconds,
        )

    async def read_message(
        self,
        message_type: type,
        topic_name: str,
        timeout_seconds: float,
    ) -> Any:
        """Read one sensor-data message outside the event-loop thread."""
        self._ensure_open()

        def read() -> Any:
            received = Event()
            captured: list[Any] = []

            def callback(message: Any) -> None:
                if not captured:
                    captured.append(message)
                    received.set()

            subscription = self._node.create_subscription(
                message_type,
                topic_name,
                qos_profile=qos_profile_sensor_data,
                callback=callback,
            )
            try:
                if not received.wait(timeout_seconds):
                    raise TimeoutError(
                        f"Timed out waiting for topic '{topic_name}'."
                    )
                return captured[0]
            finally:
                self._node.destroy_subscription(subscription)

        return await asyncio.to_thread(read)

    def now_seconds(self) -> float:
        """Return current time using the node's configured ROS clock."""
        self._ensure_open()
        return self._node.get_clock().now().nanoseconds / 1_000_000_000

    async def send_action_goal(
        self,
        action_type: type,
        action_name: str,
        goal: Any,
        server_timeout_seconds: float,
        acceptance_timeout_seconds: float,
    ) -> Any:
        """Submit an action goal outside the event-loop thread."""
        self._ensure_open()
        return await asyncio.to_thread(
            self._send_action_goal_blocking,
            action_type,
            action_name,
            goal,
            server_timeout_seconds,
            acceptance_timeout_seconds,
        )

    async def wait_action_result(self, goal_handle: Any, timeout_seconds: float) -> Any:
        """Wait for an accepted action's terminal result."""
        self._ensure_open()

        def wait() -> Any:
            future = goal_handle.get_result_async()
            completed = Event()
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(timeout_seconds):
                raise TimeoutError("Timed out waiting for action result.")
            return future.result()

        return await asyncio.to_thread(wait)

    def publish_message(self, message_type: type, topic_name: str, message: Any) -> None:
        """Publish through a cached fixed-topic publisher."""
        self._ensure_open()
        key = (message_type, topic_name)
        with self._client_lock:
            publisher = self._publishers.get(key)
            if publisher is None:
                publisher = self._node.create_publisher(message_type, topic_name, 1)
                self._publishers[key] = publisher
        publisher.publish(message)

    def close(self) -> None:
        """Stop spinning and destroy the isolated ROS context once."""
        if self._closed:
            return
        self._closed = True
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        for client in self._action_clients.values():
            client.destroy()
        for publisher in self._publishers.values():
            self._node.destroy_publisher(publisher)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        if self._context.ok():
            self._context.shutdown()

    def _call_service_blocking(
        self,
        service_type: type,
        service_name: str,
        request: Any,
        timeout_seconds: float,
    ) -> Any:
        client = self._client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=timeout_seconds):
            raise ServiceUnavailableError(
                f"Service '{service_name}' is unavailable."
            )
        future = client.call_async(request)
        completed = Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout_seconds):
            future.cancel()
            raise TimeoutError(f"Service '{service_name}' timed out.")
        return future.result()

    def _client(self, service_type: type, service_name: str) -> Any:
        key = (service_type, service_name)
        with self._client_lock:
            client = self._clients.get(key)
            if client is None:
                client = self._node.create_client(service_type, service_name)
                self._clients[key] = client
            return client

    def _send_action_goal_blocking(
        self,
        action_type: type,
        action_name: str,
        goal: Any,
        server_timeout_seconds: float,
        acceptance_timeout_seconds: float,
    ) -> Any:
        key = (action_type, action_name)
        with self._client_lock:
            client = self._action_clients.get(key)
            if client is None:
                client = ActionClient(self._node, action_type, action_name)
                self._action_clients[key] = client
        if not client.wait_for_server(timeout_sec=server_timeout_seconds):
            raise ServiceUnavailableError(
                f"Action server '{action_name}' is unavailable."
            )
        future = client.send_goal_async(goal)
        completed = Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(acceptance_timeout_seconds):
            raise TimeoutError("Timed out waiting for action goal acceptance.")
        return future.result()

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("The ROS runtime is closed.")

    def __enter__(self) -> "JazzyRosRuntime":
        return self

    def __exit__(
        self,
        exception_type: type[BaseException] | None,
        exception: BaseException | None,
        traceback: object | None,
    ) -> None:
        self.close()
