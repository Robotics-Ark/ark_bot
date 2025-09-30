# ark_bot_driver.py
from typing import Dict, Any, List
import math
import time

from ark.system.driver.robot_driver import RobotDriver
from ark.tools.log import log

# New high-level imports
from servopkg.highlevel import MultiServoController, ServoKinematics
from servopkg.bus import ServoBus

_TWO_PI = 2.0 * math.pi

class ArkBotDriver(RobotDriver):
    def __init__(self, component_name: str, component_config: Dict[str, Any] = None, sim: bool = False):
        super().__init__(component_name, component_config, sim)
        rc = self.config["real_config"]

        # Serial
        port = rc["port"]
        baud = int(rc.get("baudrate", 1_000_000))

        # Mapping
        self.joint_order = list(rc["joint_order"])
        self.motor_ids   = [int(x) for x in rc["motor_ids"]]
        assert len(self.joint_order) == len(self.motor_ids), "joint_order and motor_ids length mismatch"
        self._joint_to_sid = {jn: sid for jn, sid in zip(self.joint_order, self.motor_ids)}

        # Global kinematics defaults
        ticks_per_turn = int(rc.get("ticks_per_turn", 4096))

        # Per-motor config
        gear_ratio = {int(k): float(v) for k, v in rc.get("gear_ratios", {}).items()}
        orientations = {int(k): int(v) for k, v in rc.get("motor_orientations", {}).items()}
        pos_off_deg  = {int(k): float(v) for k, v in rc.get("hack_pos_zero_offsets_deg", {}).items()}  # legacy
        home_ticks   = {int(k): int(v)   for k, v in rc.get("home_ticks", {}).items()}
        home_loops   = {int(k): int(v)   for k, v in rc.get("home_loops", {}).items()}

        kin: Dict[int, ServoKinematics] = {}
        for sid in self.motor_ids:
            kin[sid] = ServoKinematics(
                ticks_per_turn = ticks_per_turn,
                gear_ratio     = gear_ratio.get(sid, 1.0),
                orientation    = orientations.get(sid, 1),
                pos_offset_rad = math.radians(pos_off_deg.get(sid, 0.0)),
                home_ticks     = home_ticks.get(sid, 0),
                home_loops     = home_loops.get(sid, 0),
            )

        speed_default = int(rc.get("speed_default", 133))
        acc_default   = int(rc.get("acc_default", 50))
        speed_min     = int(rc.get("speed_min", 1))
        speed_max     = int(rc.get("speed_max", 4095))

        # Build bus + controller
        self._bus = ServoBus(port, baud)
        self._ctrl = MultiServoController(
            bus=self._bus,
            servo_ids=self.motor_ids,
            kin=kin,
            speed_default=speed_default,
            acc_default=acc_default,
            speed_min=speed_min,
            speed_max=speed_max,
            init_position_mode=True,
            disable_limits=True,
        )

        log.info(f"[{component_name}] ArkBotDriver initialised on {port} @ {baud}")

    # ---------------- driver API ----------------

    def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
        """Return joint angles (radians) keyed by joint name."""
        sids = [self._joint_to_sid[j] for j in joints]
        angles = self._ctrl.read_angles_rad(sids)
        return {j: angles[self._joint_to_sid[j]] for j in joints}

    def pass_joint_group_control_cmd(self, control_mode: str, cmd: Dict[str, float], **kwargs) -> None:
        """Accepts target angles in radians by joint name; sends to controller."""
        group = kwargs.get("group_name", "arm")
        if control_mode != "position":
            log.warn(f"Only position mode implemented; ignoring control_mode={control_mode} for group {group}")
            return

        goals: Dict[int, float] = {}
        for jname, ang in cmd.items():
            sid = self._joint_to_sid[jname]
            goals[sid] = float(ang)
        self._ctrl.set_group_goals_rad(goals)
        # brief yield to let workers pick up the event (optional)
        time.sleep(0.001)

    def pass_joint_velocities(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def pass_joint_efforts(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def check_torque_status(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def shutdown_driver(self):
        self._ctrl.shutdown()
        log.info("ArkBotDriver shutdown complete")


# from typing import Dict, Any, List
# import math
# import time
# import threading

# from ark.system.driver.robot_driver import RobotDriver
# from ark.tools.log import log

# # ---- Your servo SDK ----
# from servopkg import PortHandler, sts  # expects .ReadAbsPos, ChangeMode, send_goal, ...
# # --- imports unchanged ---

# _TWO_PI = 2.0 * math.pi

# class ArkBotDriver(RobotDriver):
#     def __init__(self, component_name: str, component_config: Dict[str, Any] = None, sim: bool = False):
#         super().__init__(component_name, component_config, sim)
#         rc = self.config["real_config"]

#         # Serial, mapping, ratios (unchanged) ...
#         self.port = rc["port"]
#         self.baud = int(rc.get("baudrate", 1_000_000))

#         self.joint_order = list(rc["joint_order"])
#         self.motor_ids   = [int(x) for x in rc["motor_ids"]]
#         assert len(self.joint_order) == len(self.motor_ids)

#         self.ticks_per_turn = int(rc.get("ticks_per_turn", 4096))

#         # Gear ratios
#         gr = rc.get("gear_ratios", {})
#         self.gear_ratio = {int(k): float(v) for k, v in gr.items()}
        
#         # Motor orientations (+1 or -1)
#         mo = rc.get("motor_orientations", {})
#         self.motor_orientations = {int(k): int(v) for k, v in mo.items()}

#         # (Legacy) zero offsets kept for compatibility but NOT used in the new math
#         zo = rc.get("zero_offsets_deg", {})
#         self.zero_off_rad = {int(k): math.radians(float(v)) for k, v in zo.items()}

#         # Optional “position offset” you had before – we keep it, applied last
#         po = rc.get("hack_pos_zero_offsets_deg", {})
#         self.pos_off = {int(k): float(v) for k, v in po.items()}

#         # NEW: Home definition in loops + tick
#         ht = rc.get("home_ticks", {})
#         hl = rc.get("home_loops", {})
#         self.home_ticks = {int(k): int(v) for k, v in ht.items()}
#         self.home_loops = {int(k): int(v) for k, v in hl.items()}

#         # Precompute home_total_ticks per motor: loops*4096 + tick
#         self._home_total_ticks = {
#             sid: self.home_loops.get(sid, 0) * self.ticks_per_turn + self.home_ticks.get(sid, 0)
#             for sid in self.motor_ids
#         }

#         # Motion defaults
#         self.speed_default = int(rc.get("speed_default", 133))
#         self.acc_default   = int(rc.get("acc_default", 50))

#         self._speed_min = int(rc.get("speed_min", 1))
#         self._speed_max = int(rc.get("speed_max", 4095))    

#         # SDK + threading
#         self._port = PortHandler(self.port)
#         if not self._port.openPort():
#             raise RuntimeError(f"Failed to open port {self.port}")
#         if not self._port.setBaudRate(self.baud):
#             raise RuntimeError(f"Failed to set baudrate {self.baud}")
#         self._pkt = sts(self._port)

#         self._comm_lock = threading.Lock()
#         self._goals_ticks: Dict[int, int] = {}
#         self._events: Dict[int, threading.Event] = {}
#         self._stop = threading.Event()

#         # Seed from current absolute tick and init loop counters
#         self._previous_ticks: Dict[int, int] = {}
#         self._loop_count: Dict[int, int] = {}

#         for sid in self.motor_ids:
#             with self._comm_lock:
#                 self._pkt.ChangeMode(sid, 0)     # position mode
#                 self._pkt.ChangeMaxLimit(sid, 0) # disable limits if 0 means “none” in your SDK
#                 self._pkt.ChangeMinLimit(sid, 0)

#             cur_ticks = self._safe_read_abs_pos(sid)
#             self._goals_ticks[sid] = cur_ticks

#             # Initialize loop counters relative to current tick
#             self._previous_ticks[sid] = cur_ticks
#             self._loop_count[sid] = self.home_loops.get(sid, 0)

#             self._events[sid] = threading.Event()

#         # Workers
#         self._workers = []
#         for sid in self.motor_ids:
#             t = threading.Thread(target=self._motor_worker, args=(sid,), daemon=True, name=f"motor-{sid}")
#             t.start()
#             self._workers.append(t)

#         log.info(f"[{component_name}] ArkBotDriver initialised on {self.port} @ {self.baud}")

#     # ---------------- driver API ----------------

#     def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
#         out: Dict[str, float] = {}
#         half = self.ticks_per_turn // 2
#         for jname in joints:
#             sid   = self._sid_from_joint(jname)
#             ticks = self._safe_read_abs_pos(sid)

#             raw = ticks - self._previous_ticks[sid]
#             if   raw >  half: self._loop_count[sid] -= 1   # wrapped 4095->0
#             elif raw < -half: self._loop_count[sid] += 1   # wrapped 0->4095
#             self._previous_ticks[sid] = ticks

#             total_ticks = self._loop_count[sid] * self.ticks_per_turn + ticks
#             home_total  = self._home_total_ticks.get(sid, 0)
#             gear        = self.gear_ratio.get(sid, 1.0)
#             pos_offset  = self.pos_off.get(sid, 0.0)

#             angle = ((total_ticks - home_total) / (self.ticks_per_turn * gear)) * _TWO_PI
#             out[jname] = angle - pos_offset

#         return out



#     def pass_joint_group_control_cmd(self, control_mode: str, cmd: Dict[str, float], **kwargs) -> None:
#         group = kwargs.get("group_name", "arm")
#         if control_mode != "position":
#             log.warn(f"Only position mode is implemented; ignoring control_mode={control_mode} for group {group}")
#             return

#         for jname, target_rad in cmd.items():
#             sid = self._sid_from_joint(jname)
#             goal_total = self._angle_rad_to_total_ticks(sid, float(target_rad))
            
#             self._goals_ticks[sid] = int(round(goal_total))
#             self._events[sid].set()

#             # Debug:
#             print(f"[sid {sid}] θ={target_rad:.3f} rad -> total={goal_total:.1f} ticks -> send={goal_total}")

#         time.sleep(0.001)

#     # ---------------- helpers ----------------

#     def _angle_rad_to_total_ticks(self, sid: int, angle_rad: float) -> float:
#         """goal_total_ticks = home_total_ticks + angle_rad * (ticks_per_turn / 2π) * gear_ratio - (pos_offset in ticks)"""
#         gear = float(self.gear_ratio.get(sid, 1.0))
#         home_total = float(self._home_total_ticks.get(sid, 0))
#         pos_offset = float(self.pos_off.get(sid, 0.0))
#         ori = self.motor_orientations.get(sid, 1)
#         motor_mech_angle = (angle_rad + pos_offset) * float(ori)
#         # desired mechanical angle *from home*, expressed in ticks on the *motor* side
#         mech_ticks = (motor_mech_angle * (self.ticks_per_turn / _TWO_PI)) * gear
#         return home_total + mech_ticks

#     def _safe_read_abs_pos(self, sid: int) -> int:
#         with self._comm_lock:
#             val = self._pkt.ReadAbsPos(sid)
#         return 0 if val is None else int(val)

#     def _sid_from_joint(self, joint_name: str) -> int:
#         try:
#             idx = self.joint_order.index(joint_name)
#         except ValueError:
#             raise KeyError(f"Unknown joint name '{joint_name}'. Check real_config.joint_order.")
#         return self.motor_ids[idx]

#     def speed_for(self, sid:int) -> int:
#         gear = float(self.gear_ratio.get(sid, 1.0))
#         spd = int(round(self.speed_default) * gear )
#         return max(self._speed_min, min(self._speed_max, spd))

#     # Worker unchanged (still sends 0..4095 goals)
#     def _motor_worker(self, sid: int):
#         last_sent = None
#         while not self._stop.is_set():
#             ev = self._events[sid]
#             ev.wait(timeout=0.25)
#             if self._stop.is_set(): break
#             if ev.is_set():
#                 goal_total = self._goals_ticks[sid]
#                 ev.clear()
#                 if goal_total != last_sent:
#                     with self._comm_lock:
#                         speed = self.speed_for(sid)
#                         self._pkt.send_goal(sid, int(goal_total), speed, self.acc_default)
#                     last_sent = goal_total

#     def shutdown_driver(self):
#         self._stop.set()
#         for ev in self._events.values():
#             ev.set()
#         time.sleep(0.1)
#         try: self._port.closePort()
#         except Exception: pass
#         log.info("ArkBotDriver shutdown complete")


#     def pass_joint_velocities(self, joints: List[str]) -> Dict[str, float]:
#         raise NotImplementedError

#     def pass_joint_efforts(self, joints: List[str]) -> Dict[str, float]:
#         raise NotImplementedError

#     def check_torque_status(self, joints: List[str]) -> Dict[str, float]:
#         raise NotImplementedError
