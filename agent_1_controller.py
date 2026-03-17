import numpy as np
from typing import Dict, Any, Optional, List
import os
import time
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from grid import Grid

class MessageBus:
    CONNECTIONS_FILE = os.path.join(os.path.dirname(__file__), "conections.txt")
    MSG_PREFIX = "MSG:"

    @classmethod
    def send(cls, recipient, message):
        line = cls.MSG_PREFIX + repr(message) + "\n"
        with open(cls.CONNECTIONS_FILE, "a", encoding="utf-8") as f:
            f.write(line)

    @classmethod
    def recv_all(cls, agent_id):
        if not os.path.exists(cls.CONNECTIONS_FILE):
            return []
        with open(cls.CONNECTIONS_FILE, "r", encoding="utf-8") as f:
            lines = f.readlines()

        messages = []
        other_lines = []
        for line in lines:
            stripped = line.rstrip("\n")
            if stripped.startswith(cls.MSG_PREFIX):
                msg_str = stripped[len(cls.MSG_PREFIX):]
                try:
                    msg = eval(msg_str)
                    if isinstance(msg, dict) and msg.get("TO") == agent_id:
                        messages.append(msg)
                    else:
                        other_lines.append(line)
                except Exception:
                    other_lines.append(line)
            else:
                other_lines.append(line)

        with open(cls.CONNECTIONS_FILE, "w", encoding="utf-8") as f:
            f.writelines(other_lines)

        return messages


class Agent1Controller:
    def __init__(self):
        self.my_id = 1
        self.pixels_per_meter = 10.0
        self.agent: Any = None
        self.grid = None
        self.log_path = os.path.join(os.path.dirname(__file__), "agent1_controller_log.txt")
        self.connections_path = os.path.join(os.path.dirname(__file__), "conections.txt")
        self.waypoints = []
        self.barriers = []

        self.flag = 1
        self.last_update_time = 0.0
        self.status = "Idle"
        self.target: Optional[np.ndarray] = None
        self.waypoint_threshold: float = 15.0          # увеличено для более раннего переключения
        self.steering_gain: float = 0.8                # уменьшено для плавности
        self.waypoints_list: Optional[List[np.ndarray]] = None
        self.current_target_idx: int = 0
        self.last_pos: Optional[np.ndarray] = None
        self.all_agents_cache: Any = None
        self.pos_failure_count = 0
        self.last_critical_log_time = -1.0
        self.critical_log_interval = 1.0
        self.need_planning = True  # флаг для однократного планирования

        try:
            self.grid = Grid()
            self._log("Grid initialized successfully")
        except Exception as e:
            self._log(f"ERROR initializing Grid: {e}")

    # ---------- Служебные методы (не трогать!) ----------
    def set_agent(self, agent: Any):
        self.agent = agent
        # Никакого планирования здесь – дождёмся первого get_control

    def on_simulation_start(self):
        with open(self.log_path, "w", encoding="utf-8") as f:
            pass
        with open(self.connections_path, "w", encoding="utf-8") as f:
            pass
        self._log("the agent can write to the text")
        self._log(f"Agent {self.agent.id} controller started (IMU+LNS only mode)")

    def on_simulation_end(self):
        self._log(f"Agent {self.agent.id} controller ended")

    def _log(self, msg: str):
        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(msg + "\n")
            f.flush()

    # ---------- Остальные методы ----------
    def p2m(self, pixels):
        return pixels

    def m2p(self, meters):
        return meters

    def send_message(self, message: str):
        with open(self.connections_path, "a", encoding="utf-8") as f:
            f.write(message + "\n")

    def read_messages(self) -> str:
        try:
            with open(self.connections_path, "r", encoding="utf-8") as f:
                return f.read()
        except FileNotFoundError:
            return ""

    def request(self, recipient, msg, data):
        mid = str(time.perf_counter_ns())
        message = {
            "ID": mid,
            "TO": recipient,
            "FROM": self.my_id,
            "MSG": msg,
            "DATA": data,
        }
        MessageBus.send(int(recipient), message)
        return mid

    def listen(self):
        return MessageBus.recv_all(self.my_id)

    def answer(self, messages):
        for msg in messages:
            try:
                sender = int(msg.get("FROM"))
                task = (msg.get("MSG") or "").upper()
                data = msg.get("DATA")

                if task == "GET_POS":
                    self.request(sender, "ACK_GET_POS", self.last_pos)

                elif task == "GET_ALL_POSITIONS":
                    positions = []
                    if self.all_agents_cache is not None:
                        for agent in self.all_agents_cache.values():
                            pos = self._extract_agent_position(agent)
                            if pos is not None:
                                positions.append({
                                    "id": agent.id,
                                    "pos": pos.tolist() if isinstance(pos, np.ndarray) else pos
                                })
                    self.request(sender, "ACK_GET_ALL_POSITIONS", positions)

                elif task == "GET_STATUS":
                    self.request(
                        sender, "ACK_GET_STATUS",
                        {
                            "status": self.status,
                            "target": self.target.tolist() if self.target is not None else None
                        }
                    )

                elif task == "MOVE_TO":
                    p = None
                    if isinstance(data, dict) and "position" in data:
                        p = data["position"]
                    elif isinstance(data, (list, tuple)):
                        p = data

                    if p is None:
                        self.request(sender, "ERR_MOVE_TO", {"error": "position not provided"})
                    else:
                        target = np.array(p, dtype=float)
                        if self.last_pos is None:
                            self.request(sender, "ERR_MOVE_TO", {"error": "current position unknown"})
                        elif self.grid is None:
                            self.request(sender, "ERR_MOVE_TO", {"error": "grid not available"})
                        else:
                            path = self.grid.find_a_way(self.last_pos, target)
                            if path is None:
                                self.request(sender, "ERR_MOVE_TO", {"error": "path not found"})
                            else:
                                self.plan_path(path)
                                self.status = "moving"
                                self.request(sender, "ACK_MOVE_TO", {"result": "moving", "target": list(target)})

                elif task == "PASS_TASK":
                    p = None
                    if isinstance(data, dict) and "position" in data:
                        p = data["position"]
                    elif isinstance(data, (list, tuple)):
                        p = data
                    if p is not None:
                        self.target = np.array(p, dtype=float)
                        self.status = "moving"
                        self.request(sender, "ACK_PASS_TASK", {"accepted": True})

                elif task == "TASK_DONE":
                    self.status = "idle"
                    self.request(sender, "ACK_TASK_DONE", {"status": "idle"})

                elif task == "STOP":
                    self.target = None
                    self.status = "stopped"
                    self.request(sender, "ACK_STOP", {"status": "stopped"})

                elif task == "READY":
                    self.status = "ready"
                    self.request(sender, "ACK_READY", {"status": "ready"})

                elif task == "SET_PATH":
                    if isinstance(data, (list, tuple)) and all(
                        isinstance(p, (list, tuple)) and len(p) >= 2 for p in data
                    ):
                        waypoints_px = [np.array(p[:2], dtype=float) for p in data]
                        self.plan_path(waypoints_px)
                        self.status = "moving"
                        self.request(
                            sender, "ACK_SET_PATH",
                            {"status": "path set", "num_waypoints": len(waypoints_px)}
                        )
                        self._log(f"SET_PATH processed, {len(waypoints_px)} waypoints")
                    else:
                        self.request(sender, "ERR_SET_PATH", {"error": "invalid waypoints format"})

                else:
                    if not task.startswith("ACK"):
                        self.request(sender, "ACK_UNKNOWN", {"received": task})

            except Exception as e:
                sender_id = msg.get("FROM") if msg else None
                if sender_id is not None:
                    self.request(int(sender_id), "ERR_INTERNAL", {"error": str(e)})

    def plan_path(self, waypoints_px: List[np.ndarray]) -> None:
        self.waypoints_list = waypoints_px
        self.current_target_idx = 0
        self._log(f"Path planned with {len(waypoints_px)} waypoints (pixels)")

    def _extract_agent_position(self, agent) -> Optional[np.ndarray]:
        try:
            if agent is None:
                return None
            if isinstance(agent, dict):
                st = agent.get("state") or agent.get("State")
                if isinstance(st, dict):
                    p = st.get("position") or st.get("pos") or st.get("location")
                    if p is not None:
                        return self._coerce_to_np(p)
                for key in ("position", "pos", "location"):
                    p = agent.get(key)
                    if p is not None:
                        return self._coerce_to_np(p)
                if "x" in agent and "y" in agent:
                    try:
                        return np.array([float(agent["x"]), float(agent["y"])], dtype=float)
                    except Exception:
                        pass
                return None
            if hasattr(agent, "state"):
                st = getattr(agent, "state")
                if isinstance(st, dict):
                    p = st.get("position") or st.get("pos") or st.get("location")
                    if p is not None:
                        return self._coerce_to_np(p)
                else:
                    if hasattr(st, "position"):
                        p = st.position
                        return self._coerce_to_np(p)
                    if hasattr(st, "pos"):
                        p = st.pos
                        return self._coerce_to_np(p)
            for attr in ("position", "pos", "location"):
                if hasattr(agent, attr):
                    p = getattr(agent, attr)
                    return self._coerce_to_np(p)
            if hasattr(agent, "x") and hasattr(agent, "y"):
                return np.array([float(getattr(agent, "x")), float(getattr(agent, "y"))], dtype=float)
        except Exception as e:
            self._log(f"_extract_agent_position error: {e}")
        return None

    def _coerce_to_np(self, p) -> Optional[np.ndarray]:
        try:
            if p is None:
                return None
            if isinstance(p, dict):
                if "x" in p and "y" in p:
                    return np.array([float(p["x"]), float(p["y"])], dtype=float)
                if "0" in p and "1" in p:
                    return np.array([float(p["0"]), float(p["1"])], dtype=float)
            if isinstance(p, (list, tuple, np.ndarray)):
                if len(p) >= 2:
                    return np.array([float(p[0]), float(p[1])], dtype=float)
            if hasattr(p, "x") and hasattr(p, "y"):
                return np.array([float(p.x), float(p.y)], dtype=float)
        except Exception:
            pass
        return None

    def _get_current_position(self, state: Any, sensor_data: Dict, all_agents: Any) -> Optional[np.ndarray]:
        import ast

        def try_coerce(p):
            try:
                if p is None:
                    return None
                if isinstance(p, dict):
                    if "x" in p and "y" in p:
                        return np.array([float(p["x"]), float(p["y"])], dtype=float)
                    if "position" in p and isinstance(p["position"], (list, tuple, np.ndarray)) and len(p["position"]) >= 2:
                        return np.array([float(p["position"][0]), float(p["position"][1])], dtype=float)
                    if "pos" in p and isinstance(p["pos"], (list, tuple, np.ndarray)) and len(p["pos"]) >= 2:
                        return np.array([float(p["pos"][0]), float(p["pos"][1])], dtype=float)
                if isinstance(p, (list, tuple, np.ndarray)):
                    if len(p) >= 2:
                        return np.array([float(p[0]), float(p[1])], dtype=float)
                if hasattr(p, "x") and hasattr(p, "y"):
                    return np.array([float(p.x), float(p.y)], dtype=float)
            except Exception:
                return None
            return None

        pos_px = None
        source = None

        # --- all_agents ---
        try:
            if all_agents is not None:
                if isinstance(all_agents, dict):
                    for key, agent in all_agents.items():
                        if str(key) == str(self.my_id):
                            pos_px = self._extract_agent_position(agent)
                            if pos_px is not None:
                                source = f"all_agents[key={repr(key)}]"
                                break
                    if pos_px is None:
                        for key, agent in all_agents.items():
                            aid = None
                            if isinstance(agent, dict):
                                aid = agent.get("id") or agent.get("ID")
                            else:
                                aid = getattr(agent, "id", None)
                            if aid is not None and str(aid) == str(self.my_id):
                                pos_px = self._extract_agent_position(agent)
                                if pos_px is not None:
                                    source = f"all_agents[agent.id={aid}]"
                                    break
                elif isinstance(all_agents, (list, tuple)):
                    for agent in all_agents:
                        aid = None
                        if isinstance(agent, dict):
                            aid = agent.get("id") or agent.get("ID")
                        else:
                            aid = getattr(agent, "id", None)
                        if aid is not None and str(aid) == str(self.my_id):
                            pos_px = self._extract_agent_position(agent)
                            if pos_px is not None:
                                source = "all_agents[list_item_with_id]"
                                break
        except Exception as e:
            self._log(f"Error reading all_agents: {e}")

        # --- state ---
        if pos_px is None and state is not None:
            try:
                if isinstance(state, dict):
                    p = state.get("position") or state.get("pos") or (
                        [state.get("x"), state.get("y")] if state.get("x") is not None and state.get("y") is not None else None
                    )
                    if p is not None:
                        pos_px = try_coerce(p)
                        if pos_px is not None:
                            source = "state(dict)"
                else:
                    if hasattr(state, "position"):
                        pos_px = try_coerce(getattr(state, "position"))
                        if pos_px is not None:
                            source = "state.position"
                    elif hasattr(state, "pos"):
                        pos_px = try_coerce(getattr(state, "pos"))
                        if pos_px is not None:
                            source = "state.pos"
                    elif hasattr(state, "x") and hasattr(state, "y"):
                        pos_px = np.array([float(state.x), float(state.y)], dtype=float)
                        source = "state.x,y"
            except Exception as e:
                self._log(f"Error reading state: {e}")

        # --- sensors ---
        if pos_px is None:
            try:
                if isinstance(sensor_data, dict):
                    lns = sensor_data.get("lns") or sensor_data.get("LNS")
                    if isinstance(lns, dict):
                        for key in ("position_estimate", "position", "pos"):
                            p = lns.get(key)
                            p_c = try_coerce(p)
                            if p_c is not None:
                                pos_px = p_c
                                source = f"lns.{key}"
                                break
                    if pos_px is None:
                        gps = sensor_data.get("gps") or sensor_data.get("GPS")
                        gps_c = try_coerce(gps)
                        if gps_c is not None:
                            pos_px = gps_c
                            source = "gps"
            except Exception as e:
                self._log(f"Error reading sensors: {e}")

        # --- fallback: connections file ---
        if pos_px is None:
            try:
                path = getattr(self, "connections_path", None)
                if path and os.path.exists(path):
                    with open(path, "r", encoding="utf-8") as f:
                        lines = f.readlines()
                    for line in reversed(lines):
                        s = line.strip()
                        if not s:
                            continue
                        if s.startswith("MSG:"):
                            payload = s[len("MSG:"):]
                            try:
                                msg = ast.literal_eval(payload)
                                if isinstance(msg, dict):
                                    data = msg.get("DATA") or {}
                                    for key in ("position", "pos", "position_estimate", "pos_px"):
                                        if isinstance(data, dict) and key in data:
                                            p = try_coerce(data[key])
                                            if p is not None:
                                                pos_px = p
                                                source = f"connections_file[{msg.get('MSG')}]"
                                                break
                                    if pos_px is not None:
                                        break
                                    if isinstance(data, (list, tuple)) and len(data) >= 2:
                                        p = try_coerce(data)
                                        if p is not None:
                                            pos_px = p
                                            source = f"connections_file[{msg.get('MSG')}]"
                                            break
                                    if msg.get("MSG") and "ALL" in msg.get("MSG"):
                                        if isinstance(data, list):
                                            for ent in data:
                                                if isinstance(ent, dict) and (
                                                    "id" in ent and str(ent.get("id")) == str(self.my_id)
                                                ):
                                                    p = try_coerce(
                                                        ent.get("pos") or ent.get("position") or
                                                        ent.get("pos_px") or ent.get("position_estimate")
                                                    )
                                                    if p is not None:
                                                        pos_px = p
                                                        source = f"connections_file[{msg.get('MSG')}]"
                                                        break
                                    if pos_px is not None:
                                        break
                            except Exception:
                                continue
            except Exception as e:
                self._log(f"Error parsing connections file for position: {e}")

        if pos_px is not None:
            self._log(f"Got position {pos_px} from {source} (pixels) [extended search]")
            return pos_px
        else:
            try:
                self._log("Failed to get position from any source (extended). Snapshot:")
                self._log(f"  all_agents type: {type(all_agents).__name__ if all_agents is not None else 'None'}")
                if isinstance(all_agents, dict):
                    try:
                        self._log(f"  all_agents keys sample: {list(all_agents.keys())[:10]}")
                    except Exception:
                        pass
                elif isinstance(all_agents, (list, tuple)):
                    self._log(f"  all_agents len: {len(all_agents)}")
                self._log(f"  state type: {type(state).__name__ if state is not None else 'None'}")
                self._log(f"  sensor_data keys: {list(sensor_data.keys()) if isinstance(sensor_data, dict) else str(sensor_data)}")
                try:
                    path = getattr(self, "connections_path", None)
                    if path and os.path.exists(path):
                        with open(path, "r", encoding="utf-8") as f:
                            tail = f.readlines()[-10:]
                        self._log(f"  connections tail (last {len(tail)} lines):")
                        for L in tail:
                            self._log("    " + (L.strip()[:300]))
                except Exception:
                    pass
            except Exception:
                pass
            return None

    def get_control(self, state: Any, sensor_data: Dict[str, Any], all_agents: Any, sim_time: float):
        try:
            self._log(f"get_control called at time {sim_time}")
            self.all_agents_cache = all_agents

            messages = self.listen()
            if messages:
                self.answer(messages)

            dt = sim_time - self.last_update_time if self.last_update_time > 0 else 0.01
            self.last_update_time = sim_time

            current_pos = self._get_current_position(state, sensor_data, all_agents)

            if current_pos is None:
                self.pos_failure_count += 1
                if self.last_pos is not None:
                    current_pos = self.last_pos
                    self._log("Using last known position")
                else:
                    if self.waypoints_list and len(self.waypoints_list) > 0:
                        current_pos = self.waypoints_list[0].copy()
                        self.last_pos = current_pos
                        self._log("Position missing — assuming start at first waypoint (fallback)")
                    else:
                        now = sim_time
                        if self.last_critical_log_time < 0 or (now - self.last_critical_log_time) >= self.critical_log_interval:
                            self._log("CRITICAL: Cannot get position anywhere")
                            self.last_critical_log_time = now
                        return np.zeros(2, dtype=np.float64)

            self.last_pos = current_pos

            # Автоматическое планирование при первом получении позиции, если путь ещё не задан
            if self.need_planning and self.waypoints_list is None and self.grid is not None:
                # Ищем второго агента (id = 2) для построения пути к нему
                target_id = 2
                target_pos = None

                if all_agents is not None:
                    # Пытаемся извлечь позицию агента с target_id из all_agents
                    if isinstance(all_agents, dict):
                        for key, agent in all_agents.items():
                            aid = None
                            if isinstance(agent, dict):
                                aid = agent.get('id') or agent.get('ID')
                            else:
                                aid = getattr(agent, 'id', None) if hasattr(agent, 'id') else None
                            if aid == target_id:
                                target_pos = self._extract_agent_position(agent)
                                break
                    elif isinstance(all_agents, (list, tuple)):
                        for agent in all_agents:
                            aid = None
                            if isinstance(agent, dict):
                                aid = agent.get('id') or agent.get('ID')
                            else:
                                aid = getattr(agent, 'id', None) if hasattr(agent, 'id') else None
                            if aid == target_id:
                                target_pos = self._extract_agent_position(agent)
                                break

                if target_pos is not None:
                    self._log(f"Auto-planning path from {current_pos} to agent {target_id} at {target_pos}")
                    path = self.grid.find_a_way(current_pos, target_pos)
                    if path is not None:
                        self.plan_path(path)
                        self._log(f"Auto-planned path with {len(path)} waypoints")
                        self.status = "moving"
                    else:
                        self._log(f"Auto-planning failed: path not found to agent {target_id}")
                else:
                    self._log(f"Auto-planning: target agent {target_id} not found in all_agents")
                    pass
            imu_data = sensor_data.get("imu", {})
            orientation_euler = imu_data.get("orientation_euler", [0.0, 0.0, 0.0])
            current_angle = float(orientation_euler[2])

            throttle = 0.0
            steering = 0.0

            if self.waypoints_list is not None and self.current_target_idx < len(self.waypoints_list):
                target = self.waypoints_list[self.current_target_idx]
                direction = target - current_pos
                distance = np.linalg.norm(direction)
                self._log(f"Distance to target: {distance:.1f} px")

                if distance < self.waypoint_threshold:
                    self.current_target_idx += 1
                    self._log(f"Reached waypoint {self.current_target_idx-1} at pos {current_pos}")
                    if self.current_target_idx >= len(self.waypoints_list):
                        throttle = 0.0
                        steering = 0.0
                        self._log("All waypoints reached")
                    else:
                        target = self.waypoints_list[self.current_target_idx]
                        direction = target - current_pos
                        distance = np.linalg.norm(direction)

                if self.current_target_idx < len(self.waypoints_list):
                    desired_angle = np.arctan2(direction[1], direction[0])
                    angle_error = desired_angle - current_angle
                    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi
                    
                    max_steering = 1.0
                    steering = np.clip(self.steering_gain * angle_error, -max_steering, max_steering)
                    
                    # Если ошибка по углу слишком велика, сначала разворачиваемся на месте
                    angle_threshold = np.radians(20)  # 20 градусов
                    if abs(angle_error) > angle_threshold:
                        throttle = 0.0  # Только поворачиваем
                    else:
                        speed = 1.0
                        if distance < 2 * self.waypoint_threshold:
                            throttle = speed * (distance / (2 * self.waypoint_threshold))
                        else:
                            throttle = speed

                log_msg = f"[time={sim_time:.2f}] throttle={throttle:.2f}, steering={steering:.2f}, angle={np.degrees(current_angle):.1f}°, target_idx={self.current_target_idx}, pos=({current_pos[0]:.1f},{current_pos[1]:.1f})"
                self._log(log_msg)

            return np.array([throttle, steering], dtype=np.float64)

        except Exception as e:
            self._log(f"Ошибка в get_control: {e}")
            return np.zeros(2, dtype=np.float64)
