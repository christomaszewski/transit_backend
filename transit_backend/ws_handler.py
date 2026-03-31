"""WebSocket command dispatch — maps frontend commands to state mutations."""

from __future__ import annotations

import uuid

from .state import (
    DeviationConfig,
    KeepoutZoneState,
    ObstacleState,
    PublishConfig,
    SubscriptionConfig,
    TransitState,
    UndoManager,
    WaypointState,
)
from . import file_io


class CommandHandler:
    """Processes incoming WS commands and mutates TransitState."""

    def __init__(self, state: TransitState, undo_mgr: UndoManager, save_dir: str):
        self.state = state
        self.undo = undo_mgr
        self.save_dir = save_dir
        # Callbacks set by transit_node
        self.on_state_changed: callable = lambda: None
        self.on_config_changed: callable = lambda: None
        self.on_subscription_added: callable = lambda cfg: None
        self.on_subscription_updated: callable = lambda name, updates: None
        self.on_subscription_removed: callable = lambda name: None
        self.on_deviation_config_changed: callable = lambda: None

    def handle(self, msg: dict) -> dict | None:
        """Handle a command message. Returns optional response dict (for file I/O)."""
        cmd_type = msg.get("type", "")
        handler = self._handlers.get(cmd_type)
        if handler:
            return handler(self, msg)
        return None

    # -- Path editing --

    def _add_waypoint(self, msg: dict) -> None:
        self.undo.push(self.state)
        wp = WaypointState(
            lat=msg["lat"],
            lon=msg["lon"],
            alt=msg.get("alt", 0.0),
        )
        index = msg.get("index")
        if index is None:
            self.state.path.waypoints.append(wp)
        else:
            self.state.path.waypoints.insert(index, wp)
        self.on_state_changed()

    def _move_waypoint(self, msg: dict) -> None:
        self.undo.push(self.state)
        idx = msg["index"]
        if 0 <= idx < len(self.state.path.waypoints):
            self.state.path.waypoints[idx].lat = msg["lat"]
            self.state.path.waypoints[idx].lon = msg["lon"]
        self.on_state_changed()

    def _delete_waypoint(self, msg: dict) -> None:
        self.undo.push(self.state)
        idx = msg["index"]
        if 0 <= idx < len(self.state.path.waypoints):
            self.state.path.waypoints.pop(idx)
        self.on_state_changed()

    def _set_waypoint_time(self, msg: dict) -> None:
        self.undo.push(self.state)
        idx = msg["index"]
        if 0 <= idx < len(self.state.path.waypoints):
            self.state.path.waypoints[idx].desired_time = msg.get("desired_time")
        self.on_state_changed()

    def _set_waypoint_alt(self, msg: dict) -> None:
        self.undo.push(self.state)
        idx = msg["index"]
        if 0 <= idx < len(self.state.path.waypoints):
            self.state.path.waypoints[idx].alt = msg["alt"]
        self.on_state_changed()

    def _set_waypoint_speed(self, msg: dict) -> None:
        self.undo.push(self.state)
        idx = msg["index"]
        if 0 <= idx < len(self.state.path.waypoints):
            self.state.path.waypoints[idx].speed_to_next = msg["speed_to_next"]
        self.on_state_changed()

    def _reverse_path(self, msg: dict) -> None:
        self.undo.push(self.state)
        self.state.path.waypoints.reverse()
        self.on_state_changed()

    def _clear_path(self, msg: dict) -> None:
        self.undo.push(self.state)
        self.state.path.waypoints.clear()
        self.on_state_changed()

    # -- Environment editing --

    def _add_obstacle(self, msg: dict) -> None:
        self.undo.push(self.state)
        obs = ObstacleState(
            id=f"obs_{uuid.uuid4().hex[:6]}",
            lat=msg["lat"],
            lon=msg["lon"],
            radius=msg.get("radius", 100.0),
        )
        self.state.environment.obstacles.append(obs)
        self.on_state_changed()

    def _move_obstacle(self, msg: dict) -> None:
        self.undo.push(self.state)
        for obs in self.state.environment.obstacles:
            if obs.id == msg["id"]:
                obs.lat = msg["lat"]
                obs.lon = msg["lon"]
                break
        self.on_state_changed()

    def _resize_obstacle(self, msg: dict) -> None:
        self.undo.push(self.state)
        for obs in self.state.environment.obstacles:
            if obs.id == msg["id"]:
                obs.radius = msg["radius"]
                break
        self.on_state_changed()

    def _delete_obstacle(self, msg: dict) -> None:
        self.undo.push(self.state)
        self.state.environment.obstacles = [
            o for o in self.state.environment.obstacles if o.id != msg["id"]
        ]
        self.on_state_changed()

    def _add_keepout_zone(self, msg: dict) -> None:
        self.undo.push(self.state)
        koz = KeepoutZoneState(
            id=f"koz_{uuid.uuid4().hex[:6]}",
            vertices=msg["vertices"],
            min_altitude=msg.get("min_altitude"),
            max_altitude=msg.get("max_altitude"),
        )
        self.state.environment.keepout_zones.append(koz)
        self.on_state_changed()

    def _move_keepout_vertex(self, msg: dict) -> None:
        self.undo.push(self.state)
        for koz in self.state.environment.keepout_zones:
            if koz.id == msg["zone_id"]:
                vi = msg["vertex_index"]
                if 0 <= vi < len(koz.vertices):
                    koz.vertices[vi] = {"lat": msg["lat"], "lon": msg["lon"]}
                break
        self.on_state_changed()

    def _delete_keepout_zone(self, msg: dict) -> None:
        self.undo.push(self.state)
        self.state.environment.keepout_zones = [
            k for k in self.state.environment.keepout_zones if k.id != msg["zone_id"]
        ]
        self.on_state_changed()

    # -- Undo / Redo --

    def _undo(self, msg: dict) -> None:
        self.undo.undo(self.state)
        self.on_state_changed()

    def _redo(self, msg: dict) -> None:
        self.undo.redo(self.state)
        self.on_state_changed()

    # -- Configuration --

    def _set_publish_config(self, msg: dict) -> None:
        cfg = msg["config"]
        pc = self.state.publish_config
        for key in ("path_topic", "path_rate_hz", "path_enabled",
                     "nav_path_enabled", "nav_path_topic", "env_topic", "env_enabled"):
            if key in cfg:
                setattr(pc, key, cfg[key])
        self.on_config_changed()

    def _add_subscription(self, msg: dict) -> None:
        cfg_data = msg["config"]
        sub_cfg = SubscriptionConfig(
            name=cfg_data["name"],
            topic=cfg_data["topic"],
            msg_type=cfg_data["msg_type"],
            color=cfg_data.get("color", "#FF6B35"),
            visible=cfg_data.get("visible", True),
        )
        self.state.subscriptions[sub_cfg.name] = sub_cfg
        self.on_subscription_added(sub_cfg)

    def _update_subscription(self, msg: dict) -> None:
        name = msg["name"]
        updates = msg.get("updates", {})
        if name in self.state.subscriptions:
            sub = self.state.subscriptions[name]
            for key in ("color", "visible", "topic", "msg_type"):
                if key in updates:
                    setattr(sub, key, updates[key])
            self.on_subscription_updated(name, updates)

    def _remove_subscription(self, msg: dict) -> None:
        name = msg["name"]
        if name in self.state.subscriptions:
            del self.state.subscriptions[name]
            self.on_subscription_removed(name)

    # -- Deviation config --

    def _set_deviation_config(self, msg: dict) -> None:
        cfg = msg["config"]
        dc = self.state.deviation_config
        for key in ("mode", "reference", "targets", "show_map_visualization"):
            if key in cfg:
                setattr(dc, key, cfg[key])
        self.on_deviation_config_changed()

    # -- File I/O --

    def _save_path(self, msg: dict) -> dict:
        filename = msg["filename"]
        if not filename.endswith(".json"):
            filename += ".json"
        data = file_io.save_path(self.state, filename, self.save_dir)
        return {"type": "path_saved", "filename": filename, "download_data": data}

    def _load_path(self, msg: dict) -> dict:
        filename = msg["filename"]
        self.undo.push(self.state)
        path, env = file_io.load_path(filename, self.save_dir)
        self.state.path = path
        self.state.environment = env
        self.on_state_changed()
        return {"type": "path_loaded", "filename": filename}

    def _upload_path(self, msg: dict) -> dict:
        self.undo.push(self.state)
        path, env = file_io.parse_upload(msg["data"])
        self.state.path = path
        self.state.environment = env
        self.on_state_changed()
        return {"type": "path_loaded", "filename": "upload"}

    def _list_saved_paths(self, msg: dict) -> dict:
        files = file_io.list_saved_paths(self.save_dir)
        return {"type": "saved_paths_list", "files": files}

    # -- Dispatch table --

    _handlers = {
        "add_waypoint": _add_waypoint,
        "move_waypoint": _move_waypoint,
        "delete_waypoint": _delete_waypoint,
        "set_waypoint_time": _set_waypoint_time,
        "set_waypoint_alt": _set_waypoint_alt,
        "set_waypoint_speed": _set_waypoint_speed,
        "reverse_path": _reverse_path,
        "clear_path": _clear_path,
        "add_obstacle": _add_obstacle,
        "move_obstacle": _move_obstacle,
        "resize_obstacle": _resize_obstacle,
        "delete_obstacle": _delete_obstacle,
        "add_keepout_zone": _add_keepout_zone,
        "move_keepout_vertex": _move_keepout_vertex,
        "delete_keepout_zone": _delete_keepout_zone,
        "undo": _undo,
        "redo": _redo,
        "set_publish_config": _set_publish_config,
        "add_subscription": _add_subscription,
        "update_subscription": _update_subscription,
        "remove_subscription": _remove_subscription,
        "set_deviation_config": _set_deviation_config,
        "save_path": _save_path,
        "load_path": _load_path,
        "upload_path": _upload_path,
        "list_saved_paths": _list_saved_paths,
    }
