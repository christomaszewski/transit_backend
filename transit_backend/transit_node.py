"""Main async ROS2 node for TRANSIT — ties together WS server, HTTP server,
publishers, subscribers, and state management."""

from __future__ import annotations

import asyncio
import json
import logging
import os
import signal
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .deviation import compute_deviations
from .publishers import PublisherManager
from .state import TransitState, UndoManager
from .subscribers import SubscriptionManager
from .ws_handler import CommandHandler

logger = logging.getLogger("transit")


class TransitNode(Node):
    def __init__(self):
        super().__init__("transit_node")

        # Declare parameters
        self.declare_parameter("ws_port", 8765)
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("save_dir", "/data/paths")
        self.declare_parameter("cesium_ion_token", "")
        self.declare_parameter("tile_provider", "ion")
        self.declare_parameter("local_tile_url", "")
        self.declare_parameter("path_topic", "/transit/planned_path")
        self.declare_parameter("path_rate_hz", 1.0)
        self.declare_parameter("path_enabled", True)
        self.declare_parameter("nav_path_enabled", False)
        self.declare_parameter("nav_path_topic", "/transit/nav_path")
        self.declare_parameter("env_topic", "/transit/environment")
        self.declare_parameter("env_enabled", True)
        self.declare_parameter("frontend_dir", "")

        # Read parameters
        self.ws_port = self.get_parameter("ws_port").value
        self.http_port = self.get_parameter("http_port").value
        self.save_dir = self.get_parameter("save_dir").value
        self.frontend_dir = self.get_parameter("frontend_dir").value

        # Cesium config — env var overrides parameter
        self.cesium_ion_token = (
            os.environ.get("CESIUM_ION_TOKEN")
            or self.get_parameter("cesium_ion_token").value
        )
        self.tile_provider = self.get_parameter("tile_provider").value
        self.local_tile_url = self.get_parameter("local_tile_url").value

        # State
        self.state = TransitState()
        self.state.publish_config.path_topic = self.get_parameter("path_topic").value
        self.state.publish_config.path_rate_hz = self.get_parameter("path_rate_hz").value
        self.state.publish_config.path_enabled = self.get_parameter("path_enabled").value
        self.state.publish_config.nav_path_enabled = self.get_parameter("nav_path_enabled").value
        self.state.publish_config.nav_path_topic = self.get_parameter("nav_path_topic").value
        self.state.publish_config.env_topic = self.get_parameter("env_topic").value
        self.state.publish_config.env_enabled = self.get_parameter("env_enabled").value

        self.undo_mgr = UndoManager()

        # WS clients
        self.ws_clients: set = set()

        # Publishers
        self.pub_mgr = PublisherManager(self, self.state)

        # Subscribers
        self.sub_mgr = SubscriptionManager(self)
        self.sub_mgr.on_data_received = self._on_subscribed_data

        # Command handler
        self.cmd_handler = CommandHandler(self.state, self.undo_mgr, self.save_dir)
        self.cmd_handler.on_state_changed = self._on_state_changed
        self.cmd_handler.on_config_changed = self._on_config_changed
        self.cmd_handler.on_subscription_added = self._on_subscription_added
        self.cmd_handler.on_subscription_updated = self._on_subscription_updated
        self.cmd_handler.on_subscription_removed = self._on_subscription_removed
        self.cmd_handler.on_deviation_config_changed = self._on_deviation_config_changed

        # Deviation debounce
        self._deviation_pending = False
        self._last_deviation_time = 0.0

        # Subscribed path data cache (for deviation computation)
        self._subscribed_path_data: dict[str, list[dict]] = {}

        self.get_logger().info(
            f"TRANSIT node initialized — WS:{self.ws_port} HTTP:{self.http_port}"
        )

    # -- Callback wiring --

    def _on_state_changed(self) -> None:
        asyncio.ensure_future(self._broadcast_state())
        self.pub_mgr.publish_environment()
        self._schedule_deviation()

    def _on_config_changed(self) -> None:
        self.pub_mgr.reconfigure()
        asyncio.ensure_future(self._broadcast_state())

    def _on_subscription_added(self, cfg) -> None:
        self.sub_mgr.add(cfg)

    def _on_subscription_updated(self, name: str, updates: dict) -> None:
        if name in self.state.subscriptions:
            self.sub_mgr.update(name, self.state.subscriptions[name], updates)

    def _on_subscription_removed(self, name: str) -> None:
        self.sub_mgr.remove(name)
        self._subscribed_path_data.pop(name, None)

    def _on_deviation_config_changed(self) -> None:
        self._schedule_deviation()

    def _on_subscribed_data(self, name: str, data: dict) -> None:
        cfg = self.state.subscriptions.get(name)
        if not cfg:
            return
        if data.get("type") == "path":
            # Cache for deviation and forward to frontend
            self._subscribed_path_data[name] = data.get("waypoints", [])
            msg = {
                "type": "subscribed_path",
                "name": name,
                "topic": cfg.topic,
                "path": {
                    "path_id": data.get("path_id", ""),
                    "waypoints": data.get("waypoints", []),
                },
            }
            asyncio.ensure_future(self._broadcast(msg))
            self._schedule_deviation()
        elif data.get("type") == "environment":
            msg = {
                "type": "subscribed_environment",
                "name": name,
                "topic": cfg.topic,
                "environment": {
                    "obstacles": data.get("obstacles", []),
                    "keepout_zones": data.get("keepout_zones", []),
                },
            }
            asyncio.ensure_future(self._broadcast(msg))

    # -- Deviation --

    def _schedule_deviation(self) -> None:
        self._deviation_pending = True

    async def _deviation_loop(self) -> None:
        """Debounced deviation computation loop."""
        while rclpy.ok():
            await asyncio.sleep(0.25)
            if not self._deviation_pending:
                continue
            self._deviation_pending = False
            await self._compute_and_send_deviations()

    async def _compute_and_send_deviations(self) -> None:
        dc = self.state.deviation_config
        if not dc.targets:
            return

        # Build reference waypoints
        if dc.reference == "user_path":
            ref_wps = [w.to_dict() for w in self.state.path.waypoints]
        else:
            ref_wps = self._subscribed_path_data.get(dc.reference, [])

        if not ref_wps:
            return

        comparisons = []
        for target_name in dc.targets:
            target_wps = self._subscribed_path_data.get(target_name, [])
            if not target_wps:
                continue
            comp = compute_deviations(ref_wps, target_wps, dc.reference, target_name)
            comparisons.append(comp)

        if comparisons:
            await self._broadcast({"type": "deviation_update", "comparisons": comparisons})

    # -- WebSocket --

    async def _ws_handler(self, websocket) -> None:
        self.ws_clients.add(websocket)
        try:
            # Send cesium config
            cesium_msg = {"type": "cesium_config", "ion_token": self.cesium_ion_token}
            if self.tile_provider == "local":
                cesium_msg["tile_provider"] = "local"
                cesium_msg["tile_url"] = self.local_tile_url
            else:
                cesium_msg["tile_provider"] = "ion"
            await websocket.send(json.dumps(cesium_msg))

            # Send current state
            await websocket.send(json.dumps(self._build_state_update()))

            # Handle messages
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                    response = self.cmd_handler.handle(msg)
                    if response:
                        await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    self.get_logger().warning("Invalid JSON from WS client")
                except Exception as e:
                    self.get_logger().error(f"Error handling WS command: {e}")
        except Exception:
            pass
        finally:
            self.ws_clients.discard(websocket)

    async def _broadcast_state(self) -> None:
        await self._broadcast(self._build_state_update())

    async def _broadcast(self, msg: dict) -> None:
        if not self.ws_clients:
            return
        raw = json.dumps(msg)
        dead = set()
        for ws in self.ws_clients:
            try:
                await ws.send(raw)
            except Exception:
                dead.add(ws)
        self.ws_clients -= dead

    def _build_state_update(self) -> dict:
        return {
            "type": "state_update",
            "state": {
                "path": self.state.path.to_dict(),
                "environment": self.state.environment.to_dict(),
                "can_undo": self.undo_mgr.can_undo,
                "can_redo": self.undo_mgr.can_redo,
            },
        }

    # -- Publish loop --

    async def _publish_loop(self) -> None:
        while rclpy.ok():
            rate = self.state.publish_config.path_rate_hz
            if rate > 0:
                self.pub_mgr.publish_path()
                await asyncio.sleep(1.0 / rate)
            else:
                await asyncio.sleep(1.0)

    # -- Health check loop --

    async def _health_loop(self) -> None:
        while rclpy.ok():
            await asyncio.sleep(2.0)
            pub_status = self.pub_mgr.get_status()
            sub_status = self.sub_mgr.get_status(self.state.subscriptions)
            await self._broadcast({"type": "publish_status", "publishers": pub_status})
            await self._broadcast({"type": "subscription_status", "subscriptions": sub_status})


async def _spin_ros(node: TransitNode, executor: SingleThreadedExecutor) -> None:
    """Spin ROS2 in the asyncio event loop."""
    while rclpy.ok():
        executor.spin_once(timeout_sec=0)
        await asyncio.sleep(0.01)


async def _run(node: TransitNode, executor: SingleThreadedExecutor) -> None:
    import websockets
    from aiohttp import web

    # HTTP static file server
    app = web.Application()
    frontend_dir = node.frontend_dir
    if not frontend_dir:
        # Default: look in common locations
        for candidate in [
            "/ws/src/transit_frontend",
            os.path.join(os.path.dirname(__file__), "..", "..", "..", "transit_frontend"),
        ]:
            if os.path.isdir(candidate):
                frontend_dir = os.path.abspath(candidate)
                break

    if frontend_dir and os.path.isdir(frontend_dir):
        app.router.add_static("/", frontend_dir, show_index=True)
        node.get_logger().info(f"Serving frontend from {frontend_dir}")
    else:
        node.get_logger().warning(f"Frontend directory not found")

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", node.http_port)
    await site.start()
    node.get_logger().info(f"HTTP server on port {node.http_port}")

    # WebSocket server
    ws_server = await websockets.serve(node._ws_handler, "0.0.0.0", node.ws_port)
    node.get_logger().info(f"WebSocket server on port {node.ws_port}")

    # Run all tasks
    await asyncio.gather(
        _spin_ros(node, executor),
        node._publish_loop(),
        node._health_loop(),
        node._deviation_loop(),
    )


def main(args=None):
    rclpy.init(args=args)
    node = TransitNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(_run(node, executor))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
