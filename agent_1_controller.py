import numpy as np
from typing import Dict, Any, Optional, List
import os
import time

# =============================================================================
# MessageBus – класс для обмена структурированными сообщениями между агентами
# через файл connections.txt. Сообщения помечаются префиксом MSG:, чтобы их
# можно было отличить от обычных отладочных строк.
# =============================================================================
class MessageBus:
    # Путь к файлу-сети (общий для всех агентов)
    CONNECTIONS_FILE = os.path.join(os.path.dirname(__file__), "conections.txt")
    # Префикс, которым начинается строка, содержащая сообщение
    MSG_PREFIX = "MSG:"

    @classmethod
    def send(cls, recipient, message):
        """
        Отправляет сообщение, дописывая его в конец файла.
        recipient: ID получателя (целое число)
        message: словарь с полями ID, TO, FROM, MSG, DATA
        """
        # Преобразуем словарь в строку с помощью repr (безопасно для eval)
        line = cls.MSG_PREFIX + repr(message) + "\n"
        # Открываем файл в режиме добавления и пишем строку
        with open(cls.CONNECTIONS_FILE, "a", encoding="utf-8") as f:
            f.write(line)

    @classmethod
    def recv_all(cls, agent_id):
        """
        Читает все сообщения из файла, выбирает те, у которых TO == agent_id,
        удаляет их из файла и возвращает список выбранных сообщений.
        Строки без префикса (например, отладочные логи) остаются нетронутыми.
        """
        if not os.path.exists(cls.CONNECTIONS_FILE):
            return []

        # Читаем все строки из файла
        with open(cls.CONNECTIONS_FILE, "r", encoding="utf-8") as f:
            lines = f.readlines()

        messages = []      # сюда будем собирать сообщения для текущего агента
        other_lines = []   # сюда – все остальные строки (чужие сообщения + логи)

        for line in lines:
            stripped = line.rstrip("\n")
            if stripped.startswith(cls.MSG_PREFIX):
                # Это строка-сообщение – удаляем префикс
                msg_str = stripped[len(cls.MSG_PREFIX):]
                try:
                    # Восстанавливаем исходный словарь через eval (в данной среде безопасно)
                    msg = eval(msg_str)
                    # Проверяем, что это действительно словарь и адресовано нужному агенту
                    if isinstance(msg, dict) and msg.get("TO") == agent_id:
                        messages.append(msg)      # забираем сообщение
                    else:
                        other_lines.append(line)  # оставляем в файле
                except Exception:
                    # Если не удалось распарсить – сохраняем строку как есть
                    other_lines.append(line)
            else:
                # Это не сообщение (например, лог из send_message) – оставляем в файле
                other_lines.append(line)

        # Перезаписываем файл, оставляя только «чужие» строки (те, которые не забрали)
        with open(cls.CONNECTIONS_FILE, "w", encoding="utf-8") as f:
            f.writelines(other_lines)

        return messages

class Agent1Controller:
    def __init__(self):
        # ----- Фиксированные параметры данного агента -----
        self.my_id = 1                          # идентификатор этого агента (жёстко задан)
        self.pixels_per_meter = 10.0            # коэффициент перевода пикселей в метры (здесь не используется, оставлен для совместимости)

        self.agent: Any = None

        # ----- Пути к файлам логирования и связи -----
        self.log_path = os.path.join(os.path.dirname(__file__), "agent1_controller_log.txt")
        self.connections_path = os.path.join(os.path.dirname(__file__), "conections.txt")


        self.waypoints = []          
        self.barriers = []            

        # ----- Переменные состояния и управления -----
        self.flag = 1                             # вспомогательный флаг (можно использовать по своему усмотрению)
        self.last_update_time = 0.0                # время последнего вызова get_control (для вычисления dt)
        self.status = "Idle"                       # текущий статус агента: Idle, moving, stopped, ready и т.д.
        self.target: Optional[np.ndarray] = None   # текущая целевая точка (в пикселях), к которой движемся
        self.waypoint_threshold: float = 10.0      # расстояние (в пикселях), при котором считаем, что точка достигнута
        self.steering_gain: float = 1.5            # коэффициент усиления для рулевого управления (пропорциональный регулятор)

        self.waypoints_list: Optional[List[np.ndarray]] = None   # список путевых точек (в пикселях)
        self.current_target_idx: int = 0                         # индекс текущей целевой точки в списке

      
        self.last_pos: Optional[np.ndarray] = None    # последняя полученная позиция (в пикселях)

 
        self.all_agents_cache: Any = None

    # ---------- Методы, вызываемые симулятором (служебные) ----------
    def set_agent(self, agent: Any):
        """
        Вызывается симулятором для передачи объекта агента.
        В данной версии не используется, но оставлено для совместимости.
        """
        self.agent = agent

    def on_simulation_start(self):
        """
        Вызывается один раз при старте симуляции.
        Очищает логи и файл сообщений, а также отправляет тестовый путь самому себе.
        """
        # Очищаем файл логов и файл сообщений
        with open(self.log_path, "w", encoding="utf-8") as f:
            pass
        with open(self.connections_path, "w", encoding="utf-8") as f:
            pass
        self._log(f"Agent {self.my_id} controller started")

        # Пример: отправляем себе команду SET_PATH с двумя точками (в пикселях)
        cells = np.array([[30, 30], [900, 30]], dtype=float)
        waypoints_list = cells.tolist()
        self.request(self.my_id, "SET_PATH", waypoints_list)
        self._log("Sent SET_PATH to self")

    def on_simulation_end(self):
        """
        Вызывается при завершении симуляции.
        """
        self._log(f"Agent {self.my_id} controller ended")

    def _log(self, msg: str):
        """
        Запись отладочного сообщения в файл лога агента.
        """
        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(msg + "\n")

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
        """
        Формирует и отправляет сообщение через MessageBus.
        recipient: ID получателя
        msg: строка – тип сообщения (команда)
        data: произвольные данные (позиция, статус и т.д.)
        Возвращает уникальный идентификатор сообщения.
        """
        mid = str(time.perf_counter_ns())          # генерируем уникальный ID на основе текущего времени
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
        """
        Получает все входящие сообщения, адресованные этому агенту, через MessageBus.
        """
        return MessageBus.recv_all(self.my_id)

    def answer(self, messages):
        """
        Обрабатывает список полученных сообщений (из listen) и отправляет ответы.
        Для каждого сообщения определяется команда (task), выполняются соответствующие действия,
        и отправителю отправляется ответное сообщение (ACK_... или ERR_...).
        """
        for msg in messages:
            try:
                sender = int(msg.get("FROM"))          # ID отправителя
                task = (msg.get("MSG") or "").upper()  # команда (приводим к верхнему регистру)
                data = msg.get("DATA")                  # данные

                # ---- GET_POS - запрос текущей позиции агента ----
                # Ожидаемый DATA: любой (игнорируется)
                # Ответ: ACK_GET_POS с позицией в пикселях (список [x, y]) или None
                if task == "GET_POS":
                    # Возвращаем последнюю известную позицию (в пикселях)
                    self.request(sender, "ACK_GET_POS", self.last_pos)

                # ---- GET_ALL_POSITIONS - запрос позиций всех агентов ----
                # Ожидаемый DATA: любой (игнорируется)
                # Ответ: ACK_GET_ALL_POSITIONS со списком словарей:
                #   [{"id": id, "pos": [x, y]}, ...] (позиции в пикселях)
                elif task == "GET_ALL_POSITIONS":
                    positions = []
                    if self.all_agents_cache is not None:
                        for agent in self.all_agents_cache.values():
                            pos = self._extract_agent_position(agent)  # в пикселях
                            if pos is not None:
                                positions.append({
                                    "id": agent.id,
                                    "pos": pos.tolist() if isinstance(pos, np.ndarray) else pos
                                })
                    self.request(sender, "ACK_GET_ALL_POSITIONS", positions)

                # ---- GET_STATUS - запрос статуса агента ----
                # Ожидаемый DATA: любой (игнорируется)
                # Ответ: ACK_GET_STATUS со словарем:
                #   {"status": str, "target": [x, y] или None}
                elif task == "GET_STATUS":
                    self.request(
                        sender,
                        "ACK_GET_STATUS",
                        {
                            "status": self.status,
                            "target": self.target.tolist() if self.target is not None else None
                        }
                    )

                # ---- MOVE_TO - команда двигаться к указанной точке ----
                # DATA может быть:
                #   - словарь вида {"position": [x, y]}  (координаты в пикселях)
                #   - список/кортеж [x, y]  (координаты в пикселях)
                # Ответ: ACK_MOVE_TO со словарем {"result": "moving", "target": [x, y]} (в пикселях)
                elif task == "MOVE_TO":
                    p = None
                    if isinstance(data, dict) and "position" in data:
                        p = data["position"]
                    elif isinstance(data, (list, tuple)):
                        p = data
                    if p is None:
                        self.request(sender, "ERR_MOVE_TO", {"error": "position not provided"})
                    else:
                        self.target = np.array(p, dtype=float)          # оставляем в пикселях
                        self.status = "moving"
                        self.request(sender, "ACK_MOVE_TO", {"result": "moving", "target": list(self.target)})

                # ---- PASS_TASK - эстафетная команда (передача задачи) ----
                # DATA (опционально) может содержать точку назначения в том же формате, что и MOVE_TO
                # Если точка передана, агент начинает движение к ней.
                # Ответ: ACK_PASS_TASK со словарем {"accepted": True}
                elif task == "PASS_TASK":
                    p = None
                    if isinstance(data, dict) and "position" in data:
                        p = data["position"]
                    elif isinstance(data, (list, tuple)):
                        p = data
                    if p is not None:
                        self.target = np.array(p, dtype=float)          # оставляем в пикселях
                        self.status = "moving"
                    self.request(sender, "ACK_PASS_TASK", {"accepted": True})

                # ---- TASK_DONE - сигнал о завершении текущей задачи ----
                # DATA игнорируется
                # Ответ: ACK_TASK_DONE со словарем {"status": "idle"}
                elif task == "TASK_DONE":
                    self.status = "idle"
                    self.request(sender, "ACK_TASK_DONE", {"status": "idle"})

                # ---- STOP - команда остановки ----
                # DATA игнорируется
                # Ответ: ACK_STOP со словарем {"status": "stopped"}
                elif task == "STOP":
                    self.target = None
                    self.status = "stopped"
                    self.request(sender, "ACK_STOP", {"status": "stopped"})

                # ---- READY - команда перехода в состояние готовности ----
                # DATA игнорируется
                # Ответ: ACK_READY со словарем {"status": "ready"}
                elif task == "READY":
                    self.status = "ready"
                    self.request(sender, "ACK_READY", {"status": "ready"})

                # ---- SET_PATH - задание маршрута (списка путевых точек) ----
                # DATA должен быть списком списков/кортежей: [[x1,y1], [x2,y2], ...] (координаты в пикселях)
                # Ответ: ACK_SET_PATH со словарем {"status": "path set", "num_waypoints": n}
                elif task == "SET_PATH":
                    if isinstance(data, (list, tuple)) and all(isinstance(p, (list, tuple)) and len(p) >= 2 for p in data):
                        waypoints_px = [np.array(p[:2], dtype=float) for p in data]
                        self.plan_path(waypoints_px)
                        self.status = "moving"
                        self.request(sender, "ACK_SET_PATH", {"status": "path set", "num_waypoints": len(waypoints_px)})
                        self._log(f"SET_PATH processed, {len(waypoints_px)} waypoints")
                    else:
                        self.request(sender, "ERR_SET_PATH", {"error": "invalid waypoints format"})

                # ---- Неизвестная команда ----
                else:
                    self.request(sender, "ACK_UNKNOWN", {"received": task})

            except Exception as e:
                # Внутренняя ошибка при обработке – отправляем ERR_INTERNAL отправителю
                sender_id = msg.get("FROM") if msg else None
                if sender_id is not None:
                    self.request(int(sender_id), "ERR_INTERNAL", {"error": str(e)})

    # ---------- Планирование пути ----------
    def plan_path(self, waypoints_px: List[np.ndarray]) -> None:
        """
        Сохраняет путь, состоящий из точек в пикселях.
        waypoints_px – список numpy-массивов [x, y] (пиксели).
        После вызова этого метода агент начинает двигаться от первой точки к последней.
        """
        self.waypoints_list = waypoints_px
        self.current_target_idx = 0
        self._log(f"Path planned with {len(waypoints_px)} waypoints (pixels)")

    # ---------- Извлечение позиции агента из его объекта (в пикселях) ----------
    def _extract_agent_position(self, agent) -> Optional[np.ndarray]:
        """
        Пытается извлечь позицию агента (в пикселях) из его объекта,
        проверяя различные возможные атрибуты (state.position, position, pos, location).
        Возвращает numpy array [x, y] или None.
        """
        try:
            # Проверяем agent.state.position
            if hasattr(agent, 'state') and hasattr(agent.state, 'position'):
                p = agent.state.position
                if isinstance(p, (list, tuple, np.ndarray)) and len(p) >= 2:
                    return np.array(p[:2], dtype=float)
            # Проверяем agent.position
            if hasattr(agent, 'position'):
                p = agent.position
                if hasattr(p, 'x') and hasattr(p, 'y'):
                    return np.array([p.x, p.y], dtype=float)
                if isinstance(p, (list, tuple, np.ndarray)) and len(p) >= 2:
                    return np.array(p[:2], dtype=float)
            # Проверяем agent.pos
            if hasattr(agent, 'pos'):
                p = agent.pos
                if hasattr(p, 'x') and hasattr(p, 'y'):
                    return np.array([p.x, p.y], dtype=float)
                if isinstance(p, (list, tuple, np.ndarray)) and len(p) >= 2:
                    return np.array(p[:2], dtype=float)
            # Проверяем agent.location
            if hasattr(agent, 'location'):
                p = agent.location
                if hasattr(p, 'x') and hasattr(p, 'y'):
                    return np.array([p.x, p.y], dtype=float)
                if isinstance(p, (list, tuple, np.ndarray)) and len(p) >= 2:
                    return np.array(p[:2], dtype=float)
        except Exception:
            pass
        return None

    # ---------- Получение текущей позиции агента (в пикселях) из нескольких источников ----------
    def _get_current_position(self, state: Any, sensor_data: Dict, all_agents: Any) -> Optional[np.ndarray]:
        """
        Пытается получить текущие координаты агента из различных источников:
        1) Из all_agents (находит себя по self.my_id и извлекает позицию)
        2) Из объекта state (различные атрибуты)
        3) Из lns (локальная навигационная система) в sensor_data
        4) Из gps в sensor_data
        Возвращает позицию в пикселях или None.
        """
        pos_px = None
        source = None

        # 1. Пробуем all_agents – находим себя по self.my_id
        if all_agents is not None:
            try:
                if self.my_id in all_agents:
                    agent = all_agents[self.my_id]
                    pos_px = self._extract_agent_position(agent)
                    if pos_px is not None:
                        source = "all_agents"
            except Exception as e:
                self._log(f"Error getting self from all_agents: {e}")

        # 2. Пробуем state (если позиция ещё не найдена)
        if pos_px is None and state is not None:
            if hasattr(state, 'position'):
                p = state.position
                if hasattr(p, 'x') and hasattr(p, 'y'):
                    pos_px = np.array([p.x, p.y], dtype=float)
                    source = "state.position"
                elif isinstance(p, (list, tuple, np.ndarray)) and len(p) >= 2:
                    pos_px = np.array(p[:2], dtype=float)
                    source = "state.position"
            if pos_px is None and hasattr(state, 'pos'):
                p = state.pos
                if hasattr(p, 'x') and hasattr(p, 'y'):
                    pos_px = np.array([p.x, p.y], dtype=float)
                    source = "state.pos"
                elif isinstance(p, (list, tuple, np.ndarray)) and len(p) >= 2:
                    pos_px = np.array(p[:2], dtype=float)
                    source = "state.pos"
            if pos_px is None and hasattr(state, 'x') and hasattr(state, 'y'):
                pos_px = np.array([state.x, state.y], dtype=float)
                source = "state.x, state.y"

        # 3. Пробуем LNS (локальная навигационная система)
        if pos_px is None:
            lns_data = sensor_data.get("lns", {})
            pos_estimate = lns_data.get("position_estimate")
            if pos_estimate is not None and len(pos_estimate) >= 2:
                pos_px = np.array(pos_estimate[:2], dtype=float)
                source = "lns.position_estimate"

        # 4. Пробуем GPS (запасной вариант)
        if pos_px is None:
            gps = sensor_data.get("gps")
            if gps is not None:
                if hasattr(gps, 'x') and hasattr(gps, 'y'):
                    pos_px = np.array([gps.x, gps.y], dtype=float)
                    source = "gps"
                elif isinstance(gps, (list, tuple, np.ndarray)) and len(gps) >= 2:
                    pos_px = np.array(gps[:2], dtype=float)
                    source = "gps"

        if pos_px is not None:
            self._log(f"Got position {pos_px} from {source} (pixels)")
            return pos_px
        else:
            self._log("Failed to get position from any source")
            return None

    # ---------- Основной метод управления, вызываемый симулятором каждый тик (0.01 с) ----------
    def get_control(self, state: Any, sensor_data: Dict[str, Any], all_agents: Any, sim_time: float):
        """
        Вызывается на каждом шаге симуляции. Должен вернуть массив [throttle, steering].
        throttle – коэффициент мощности движения (от -1 до 1),
        steering – коэффициент поворота (от -1 до 1).

        Логика управления:
        - Если задан путь (self.waypoints_list), агент движется последовательно от точки к точке.
        - Текущая цель – waypoints_list[current_target_idx].
        - Расстояние до цели вычисляется, и если оно меньше порога (waypoint_threshold),
          агент переключается на следующую точку.
        - Желаемое направление вычисляется как угол вектора от текущей позиции к цели.
        - Ошибка угла (разница между желаемым и текущим углом) используется для пропорционального
          управления поворотом (steering_gain * angle_error).
        - Скорость (throttle) устанавливается максимальной, если до цели далеко, и линейно
          уменьшается при приближении к порогу 2*waypoint_threshold для плавной остановки.

        Параметры:
          state – состояние агента (может содержать позицию и т.д.)
          sensor_data – словарь с данными от сенсоров (включая imu, gps, lns и т.д.)
          all_agents – информация обо всех агентах (обычно словарь {id: agent})
          sim_time – текущее время симуляции в секундах
        """
        try:
            # Сохраняем all_agents в кеш для использования в answer (например, для GET_ALL_POSITIONS)
            self.all_agents_cache = all_agents

            # Получаем и обрабатываем все входящие сообщения
            messages = self.listen()
            if messages:
                self.answer(messages)

            # Вычисляем дельту времени (пока не используется, но может пригодиться)
            dt = sim_time - self.last_update_time if self.last_update_time > 0 else 0.01
            self.last_update_time = sim_time

            # Получаем текущую позицию (в пикселях) из всех доступных источников
            current_pos = self._get_current_position(state, sensor_data, all_agents)

            # Если позицию получить не удалось, используем последнюю известную
            if current_pos is None:
                if self.last_pos is not None:
                    current_pos = self.last_pos
                    self._log("Using last known position")
                else:
                    # Совсем нет информации – останавливаемся
                    self._log("CRITICAL: Cannot get position anywhere")
                    return np.zeros(2, dtype=np.float64)

            self.last_pos = current_pos

            # Получаем текущий угол поворота из IMU (в радианах)
            imu_data = sensor_data.get("imu", {})
            orientation_euler = imu_data.get("orientation_euler", [0.0, 0.0, 0.0])
            current_angle = float(orientation_euler[2])

            throttle = 0.0
            steering = 0.0

            # ---------- Логика движения по маршруту ----------
            if self.waypoints_list is not None and self.current_target_idx < len(self.waypoints_list):
                # Берём текущую целевую точку
                target = self.waypoints_list[self.current_target_idx]
                # Вектор от текущей позиции к цели
                direction = target - current_pos
                # Расстояние до цели
                distance = np.linalg.norm(direction)
                self._log(f"Distance to target: {distance:.1f} px")

                # Проверка достижения текущей цели
                if distance < self.waypoint_threshold:
                    # Точка достигнута – переходим к следующей
                    self.current_target_idx += 1
                    self._log(f"Reached waypoint {self.current_target_idx-1} at pos {current_pos}")
                    if self.current_target_idx >= len(self.waypoints_list):
                        # Если это была последняя точка, останавливаемся
                        throttle = 0.0
                        steering = 0.0
                        self._log("All waypoints reached")
                    else:
                        # Берём следующую точку и пересчитываем направление
                        target = self.waypoints_list[self.current_target_idx]
                        direction = target - current_pos
                        distance = np.linalg.norm(direction)

                if self.current_target_idx < len(self.waypoints_list):
                    # Вычисляем желаемый угол направления на цель
                    desired_angle = np.arctan2(direction[1], direction[0])
                    # Ошибка угла (разница между желаемым и текущим)
                    angle_error = desired_angle - current_angle
                    # Приводим ошибку к интервалу [-π, π]
                    angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi

                    # Пропорциональное управление поворотом (steering)
                    max_steering = 1.0
                    steering = np.clip(self.steering_gain * angle_error, -max_steering, max_steering)

                    # Управление скоростью (throttle)
                    speed = 1.0
                    if distance < 2 * self.waypoint_threshold:
                        # Если близко к цели, плавно замедляемся
                        throttle = speed * (distance / (2 * self.waypoint_threshold))
                    else:
                        # Иначе движемся с максимальной скоростью
                        throttle = speed

            # Логирование и отправка отладочной информации
            log_msg = f"[time={sim_time:.2f}] throttle={throttle:.2f}, steering={steering:.2f}, angle={np.degrees(current_angle):.1f}°, target_idx={self.current_target_idx}, pos=({current_pos[0]:.1f},{current_pos[1]:.1f})"
            self._log(log_msg)
            self.send_message(f"Agent{self.my_id}: cmd=({throttle:.2f}, {steering:.2f}), time={sim_time:.2f}")

            # Возвращаем команды управления
            return np.array([throttle, steering], dtype=np.float64)

        except Exception as e:
            # В случае любой ошибки логируем и возвращаем нулевое управление
            self._log(f"Ошибка в get_control: {e}")
            return np.zeros(2, dtype=np.float64)
