# ark_bot_driver.py
from typing import Dict, Any, List
import math
import time
import threading

from ark.system.driver.robot_driver import RobotDriver
from ark.tools.log import log

# ---- Your servo SDK ----
from servopkg import PortHandler, sts  # expects .ReadAbsPos, ChangeMode, send_goal, ...
# --- imports unchanged ---

_TWO_PI = 2.0 * math.pi

class ArkBotDriver(RobotDriver):
    def __init__(self, component_name: str, component_config: Dict[str, Any] = None, sim: bool = False):
        super().__init__(component_name, component_config, sim)
        rc = self.config["real_config"]

        # Serial, mapping, ratios (unchanged) ...
        self.port = rc["port"]
        self.baud = int(rc.get("baudrate", 1_000_000))

        self.joint_order = list(rc["joint_order"])
        self.motor_ids   = [int(x) for x in rc["motor_ids"]]
        assert len(self.joint_order) == len(self.motor_ids)

        self.ticks_per_turn = float(rc.get("ticks_per_turn", 4096))

        # Gear ratios
        gr = rc.get("gear_ratios", {})
        self.gear_ratio = {int(k): float(v) for k, v in gr.items()}

        # (Legacy) zero offsets kept for compatibility but NOT used in the new math
        zo = rc.get("zero_offsets_deg", {})
        self.zero_off_rad = {int(k): math.radians(float(v)) for k, v in zo.items()}

        # Optional “position offset” you had before – we keep it, applied last
        po = rc.get("hack_pos_zero_offsets_deg", {})
        self.pos_off = {int(k): float(v) for k, v in po.items()}

        # NEW: Home definition in loops + tick
        ht = rc.get("home_ticks", {})
        hl = rc.get("home_loops", {})
        self.home_ticks = {int(k): int(v) for k, v in ht.items()}
        self.home_loops = {int(k): int(v) for k, v in hl.items()}

        # Precompute home_total_ticks per motor: loops*4096 + tick
        self._home_total_ticks = {
            sid: self.home_loops.get(sid, 0) * int(self.ticks_per_turn) + self.home_ticks.get(sid, 0)
            for sid in self.motor_ids
        }

        # Motion defaults
        self.speed_default = int(rc.get("speed_default", 133))
        self.acc_default   = int(rc.get("acc_default", 50))

        self._speed_min = int(rc.get("speed_min", 1))
        self._speed_max = int(rc.get("speed_max", 4095))    

        # SDK + threading
        self._port = PortHandler(self.port)
        if not self._port.openPort():
            raise RuntimeError(f"Failed to open port {self.port}")
        if not self._port.setBaudRate(self.baud):
            raise RuntimeError(f"Failed to set baudrate {self.baud}")
        self._pkt = sts(self._port)

        self._comm_lock = threading.Lock()
        self._goals_ticks: Dict[int, int] = {}
        self._events: Dict[int, threading.Event] = {}
        self._stop = threading.Event()

        # Seed from current absolute tick and init loop counters
        self._previous_ticks: Dict[int, int] = {}
        self._loop_count: Dict[int, int] = {}

        for sid in self.motor_ids:
            with self._comm_lock:
                self._pkt.ChangeMode(sid, 0)     # position mode
                self._pkt.ChangeMaxLimit(sid, 0) # disable limits if 0 means “none” in your SDK
                self._pkt.ChangeMinLimit(sid, 0)

            cur_ticks = self._safe_read_abs_pos(sid)
            self._goals_ticks[sid] = cur_ticks

            # Initialize loop counters relative to current tick
            self._previous_ticks[sid] = cur_ticks
            self._loop_count[sid] = 0

            self._events[sid] = threading.Event()

        # Workers
        self._workers = []
        for sid in self.motor_ids:
            t = threading.Thread(target=self._motor_worker, args=(sid,), daemon=True, name=f"motor-{sid}")
            t.start()
            self._workers.append(t)

        log.info(f"[{component_name}] ArkBotDriver initialised on {self.port} @ {self.baud}")

    # ---------------- driver API ----------------

    def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
        """
        Reads raw ticks (0..4095) and loop wraps, converts to radians relative to HOME (loops+tick).
        angle_rad = ((total_ticks_now - home_total_ticks) / (ticks_per_turn * gear_ratio)) * 2π - pos_offset
        """
        out: Dict[str, float] = {}
        for jname in joints:
            sid = self._sid_from_joint(jname)
            ticks = self._safe_read_abs_pos(sid)

            # Update wrap/loop counter
            delta = ticks - self._previous_ticks[sid]
            if   delta >  self.ticks_per_turn / 2: self._loop_count[sid] -= 1
            elif delta < -self.ticks_per_turn / 2: self._loop_count[sid] += 1
            self._previous_ticks[sid] = ticks

            total_ticks = int(self._loop_count[sid] * self.ticks_per_turn + ticks)

            gear = self.gear_ratio.get(sid, 1.0)
            pos_offset = self.pos_off.get(sid, 0.0)  # (rad) optional last-mile tweak

            home_total = self._home_total_ticks.get(sid, 0)

            mech_turns_rad = ( (total_ticks - home_total) / (self.ticks_per_turn * gear) ) * _TWO_PI
            out[jname] = mech_turns_rad - pos_offset

            # You can print for debugging:
            # print(f"[sid {sid}] ticks={ticks} loops={self._loop_count[sid]} total={total_ticks} -> {out[jname]:.3f} rad")

        return out
    
    # def pass_joint_positions(self, joints: List[str]) -> List[float]:
    #     """
    #     Return a list of joint angles [rad] in the same order as `joints`.
    #     Uses absolute ticks + wrap counting relative to HOME.
    #     """
    #     vals: List[float] = []
    #     for jname in joints:
    #         sid = self._sid_from_joint(jname)
    #         ticks = self._safe_read_abs_pos(sid)

    #         # wrap/loop counter
    #         delta = ticks - self._previous_ticks[sid]
    #         if   delta >  self.ticks_per_turn / 2: self._loop_count[sid] -= 1
    #         elif delta < -self.ticks_per_turn / 2: self._loop_count[sid] += 1
    #         self._previous_ticks[sid] = ticks

    #         total_ticks = int(self._loop_count[sid] * self.ticks_per_turn + ticks)

    #         gear       = self.gear_ratio.get(sid, 1.0)
    #         pos_offset = self.pos_off.get(sid, 0.0)              # (rad)
    #         home_total = self._home_total_ticks.get(sid, 0)

    #         mech_turns_rad = ((total_ticks - home_total) / (self.ticks_per_turn * gear)) * _TWO_PI
    #         vals.append(mech_turns_rad - pos_offset)

    #     return vals


    def pass_joint_group_control_cmd(self, control_mode: str, cmd: Dict[str, float], **kwargs) -> None:
        group = kwargs.get("group_name", "arm")
        if control_mode != "position":
            log.warn(f"Only position mode is implemented; ignoring control_mode={control_mode} for group {group}")
            return

        for jname, target_rad in cmd.items():
            sid = self._sid_from_joint(jname)
            goal_total = self._angle_rad_to_total_ticks(sid, float(target_rad))

            self._goals_ticks[sid] = int(round(goal_total))
            self._events[sid].set()

            # Debug:
            print(f"[sid {sid}] θ={target_rad:.3f} rad -> total={goal_total:.1f} ticks -> send={goal_total}")

        time.sleep(0.001)

    # ---------------- helpers ----------------

    def _angle_rad_to_total_ticks(self, sid: int, angle_rad: float) -> float:
        """goal_total_ticks = home_total_ticks + angle_rad * (ticks_per_turn / 2π) * gear_ratio - (pos_offset in ticks)"""
        gear = float(self.gear_ratio.get(sid, 1.0))
        home_total = float(self._home_total_ticks.get(sid, 0))
        pos_offset = float(self.pos_off.get(sid, 0.0))
        # desired mechanical angle *from home*, expressed in ticks on the *motor* side
        mech_ticks = ( (angle_rad + pos_offset) * (self.ticks_per_turn / _TWO_PI) ) * gear
        return home_total + mech_ticks

    def _safe_read_abs_pos(self, sid: int) -> int:
        with self._comm_lock:
            val = self._pkt.ReadAbsPos(sid)
        return 0 if val is None else int(val)

    def _sid_from_joint(self, joint_name: str) -> int:
        try:
            idx = self.joint_order.index(joint_name)
        except ValueError:
            raise KeyError(f"Unknown joint name '{joint_name}'. Check real_config.joint_order.")
        return self.motor_ids[idx]

    def speed_for(self, sid:int) -> int:
        gear = float(self.gear_ratio.get(sid, 1.0))
        spd = int(round(self.speed_default) * gear )
        return max(self._speed_min, min(self._speed_max, spd))

    # Worker unchanged (still sends 0..4095 goals)
    def _motor_worker(self, sid: int):
        last_sent = None
        while not self._stop.is_set():
            ev = self._events[sid]
            ev.wait(timeout=0.25)
            if self._stop.is_set(): break
            if ev.is_set():
                goal_total = self._goals_ticks[sid]
                ev.clear()
                if goal_total != last_sent:
                    with self._comm_lock:
                        speed = self.speed_for(sid)
                        self._pkt.send_goal(sid, int(goal_total), speed, self.acc_default)
                    last_sent = goal_total

    def shutdown_driver(self):
        self._stop.set()
        for ev in self._events.values():
            ev.set()
        time.sleep(0.1)
        try: self._port.closePort()
        except Exception: pass
        log.info("ArkBotDriver shutdown complete")

# _TWO_PI = 2.0 * math.pi

# class ArkBotDriver(RobotDriver):
#     """
#     ARK driver that talks to your the ark bot motors using servopkg.

#     Notes:
#       - This driver publishes/accepts joint angles in *radians*.
#       - It convert radians <-> absolute encoder ticks via gear_ratio & zero_offset.

#     """

#     def __init__(self, component_name: str, component_config: Dict[str, Any] = None, sim: bool = False):
#         super().__init__(component_name, component_config, sim)
        
#         rc = self.config["real_config"]

#         # Serial
#         self.port = rc["port"]
#         self.baud = int(rc.get("baudrate", 1_000_000))

#         # Joint mapping
#         self.joint_order: List[str] = list(rc["joint_order"])        # ordered joint names
#         self.motor_ids: List[int] = [int(x) for x in rc["motor_ids"]]# same length/order as joint_order
#         assert len(self.joint_order) == len(self.motor_ids), "joint_order and motor_ids length mismatch"

#         # Kinematic conversion
#         self.ticks_per_turn = float(rc.get("ticks_per_turn", 4096))
#         gr = rc.get("gear_ratios", {})
#         self.gear_ratio = {int(k): float(v) for k, v in gr.items()}

#         zo = rc.get("zero_offsets_deg", {})
#         self.zero_off_rad = {int(k): math.radians(float(v)) for k, v in zo.items()}

#         #position off set
#         po = rc.get("hack_pos_zero_offsets_deg", {})
#         self.pos_off = {int(k): float(v) for k, v in po.items()}

#         # Home def in loops and ticks
#         ht = rc.get("home_ticks", {})
#         hl = rc.get("home_loops", {})
#         self.home_ticks = {int(k): int(v) for k, v in ht.items()}
#         self.home_loops = {int(k): int(v) for k, v in hl.items()}

#         # Precompute home_total_ticks per motor: loops*4096 + tick
#         self._home_total_ticks = {
#             sid: self.home_loops.get(sid, 0) * int(self.ticks_per_turn) + self.home_ticks.get(sid, 0)
#             for sid in self.motor_ids
#         }

#         # Motion defaults
#         self.speed_default = int(rc.get("speed_default", 1200))
#         self.acc_default   = int(rc.get("acc_default", 50))

#         # SDK + threading
#         self._port = PortHandler(self.port)
#         if not self._port.openPort():
#             raise RuntimeError(f"Failed to open port {self.port}")
#         if not self._port.setBaudRate(self.baud):
#             raise RuntimeError(f"Failed to set baudrate {self.baud}")

#         self._pkt = sts(self._port)

#         # Protect serial access
#         self._comm_lock = threading.Lock()

#         # Current goals in ticks (absolute), per motor-id
#         self._goals_ticks: Dict[int, int] = {}
#         # Per-motor event to wake worker
#         self._events: Dict[int, threading.Event] = {}
#         # Stop flag
#         self._stop = threading.Event()

#         # Seed from current absolute tick and init loop counters
#         self._previous_ticks: Dict[int, int] = {}
#         self._loop_count: Dict[int, int] = {}

#         # Configure motors and seed goals with current abs pos
#         for sid in self.motor_ids:
#             with self._comm_lock:
#                 self._pkt.ChangeMode(sid, 0)       # position mode
#                 self._pkt.ChangeMaxLimit(sid, 0)   # disable lib-side limits if 0 means no limit
#                 self._pkt.ChangeMinLimit(sid, 0)
#             cur_ticks = self._safe_read_abs_pos(sid)
#             self._goals_ticks[sid] = cur_ticks
#             self._events[sid] = threading.Event()
        
#         # self._pkt.SetMiddle(4)
#         # self._pkt.SetMiddle(5)
#         # self._pkt.SetMiddle(6)
#         # print("saiudhoasudhshadihsad")

#         # Workers
#         self._workers: List[threading.Thread] = []
#         for sid in self.motor_ids:
#             t = threading.Thread(target=self._motor_worker, args=(sid,), daemon=True, name=f"motor-{sid}")
#             t.start()
#             self._workers.append(t)

#         log.info(f"[{component_name}] ArkBotDriver initialised on {self.port} @ {self.baud}")


    
#     # ======================
#     # Core Functions
#     # ======================

#     def shutdown_driver(self):
#         # called by ARK when stopping
#         self._stop.set()
#         for ev in self._events.values():
#             ev.set()
#         time.sleep(0.1)
#         try:
#             self._port.closePort()
#         except Exception:
#             pass
#         log.info("ArkBotDriver shutdown complete")

#     # ======================
#     # Driver Functions
#     # ======================
    
#     # def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
#     #     """
#     #     Return {joint_name: angle_in_radians} for the requested joints.
#     #     """
#     #     out: Dict[str, float] = {}
#     #     for jname in joints:
#     #         sid = self._sid_from_joint(jname)
#     #         ticks = self._safe_read_abs_pos(sid)
#     #         ang = self._ticks_to_rad(sid, ticks)
#     #         out[jname] = ang
#     #         # print(out)
#     #     return out


#     # def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
#     #     """
#     #     Return {joint_name: angle_in_radians} for the requested joints, accounting for zero offsets.
#     #     This function will compute joint positions based on encoder ticks, gear ratios, and zero offsets.
#     #     """
#     #     out: Dict[str, float] = {}

    

#     #     for jname in joints:
#     #         sid = self._sid_from_joint(jname)
            
#     #         # Read the absolute position in ticks
#     #         ticks = self._safe_read_abs_pos(sid)

#     #         # Get the zero offset in radians (from zero_off_rad)
#     #         zero_offset_rad = self.zero_off_rad.get(sid, 0.0)
#     #         # get the position offset 
#     #         position_offset = self.pos_off.get(sid, 0.0)
#     #         # Get the gear ratio for the joint
#     #         gear_ratio = self.gear_ratio.get(sid, 1.0)

#     #         # Convert ticks to radians (first apply the gear ratio, then the zero offset)
#     #         # Formula: angle = (ticks / ticks_per_turn) * (2π) / gear_ratio
#     #         angle_rad = (ticks / self.ticks_per_turn) * (2.0 * math.pi) / gear_ratio
            
            
#     #         # Apply the zero offset to the computed angle
#     #         angle_with_offset = angle_rad + zero_offset_rad

#     #         angle_with_position_offset = angle_with_offset - position_offset
#     #         # Store the final computed angle
#     #         out[jname] = angle_with_position_offset
#     #         # print("sid:", sid, "ticks:", ticks, "angle_rad:", angle_rad, "zero_offset_rad:", zero_offset_rad, "angle_with_offset:", angle_with_offset, "final_angle:", angle_with_position_offset)

#     #     return out

#     def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
#         """
#         Return {joint_name: angle_in_radians} for the requested joints, accounting for zero offsets.
#         This function will compute joint positions based on encoder ticks, gear ratios, and zero offsets.
#         """
#         out: Dict[str, float] = {}

#         for jname in joints:
#             sid = self._sid_from_joint(jname)
            
#             # Read the absolute position in ticks
#             ticks = self._safe_read_abs_pos(sid)

#             # Get the zero offset in radians (from zero_off_rad)
#             zero_offset_rad = self.zero_off_rad.get(sid, 0.0)
#             # Get the position offset
#             position_offset = self.pos_off.get(sid, 0.0)
#             # Get the gear ratio for the joint
#             gear_ratio = self.gear_ratio.get(sid, 1.0)

#             home_total = self._home_total_ticks.get(sid, 0)

#             # Calculate the number of full loops (revolutions) and the "delta ticks"
#             delta_ticks = ticks - self._previous_ticks[sid]

#             if delta_ticks > 2048:  # If the delta exceeds half of the full rotation (4096 ticks)
#                 self._loop_count[sid] -= 1  # This means the motor has passed a full rotation.
#             elif delta_ticks < -2048:  # Handle negative deltas indicating a rollover in the opposite direction
#                 self._loop_count[sid] += 1

#             # Update the previous ticks for the next loop
#             self._previous_ticks[sid] = ticks
            
#             # The total ticks considering the number of loops
#             total_ticks = ticks + self._loop_count[sid] * 4096
#             print("sid:", sid, "ticks:", ticks, "total_ticks:", total_ticks, "delta_ticks:", delta_ticks, "loops:", self._loop_count[sid])
#             # Convert ticks to radians (first apply the gear ratio, then the zero offset)
#             angle_rad = (total_ticks / self.ticks_per_turn) * (2.0 * math.pi) / gear_ratio
            
#             # Apply the zero offset to the computed angle
#             angle_with_offset = angle_rad + zero_offset_rad

#             # Apply the position offset (if any)
#             angle_with_position_offset = angle_with_offset - position_offset

#             # Store the final computed angle
#             out[jname] = angle_with_position_offset

#         return out



    def pass_joint_velocities(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def pass_joint_efforts(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def check_torque_status(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

#     # ======================
#     # Control Functions
#     # ======================

#     def pass_joint_group_control_cmd(self, control_mode: str, cmd: Dict[str, float], **kwargs) -> None:

#         group = kwargs.get("group_name", "arm")
#         print(f"Received joint group command for group '{group}': {cmd}")
#         if control_mode == "position":
#             # Convert each joint target rad -> ticks and queue for its worker
#             for jname in cmd.keys():
#                 sid = self._sid_from_joint(jname)
  
#                 target_rad = float(cmd[jname])
#                 target_ticks = self._rad_to_ticks(sid, target_rad)
               
#                 # update goal and wake worker
#                 self._goals_ticks[sid] = int(target_ticks)
#                 self._events[sid].set()
#                 print("id", sid, "target_rad:", target_rad, "target_ticks:", target_ticks)

#         elif control_mode == "velocity":
#             log.warn(f"Velocity mode not implemented; ignoring command for group {group}")
#         else:
#             log.error(f"Unsupported control_mode: {control_mode}")

#         time.sleep(0.001)  # tiny yield

#     def pass_cartesian_control_cmd(self, control_mode, position, quaternion, **kwargs) -> None:
#         log.warn("Cartesian control not implemented in ArkBotDriver; ignoring.")

#     # -------------- workers & IO helpers --------------
#     def _motor_worker(self, sid: int):
#         last_sent = None
#         while not self._stop.is_set():
#             ev = self._events[sid]
#             ev.wait(timeout=0.25)
#             if self._stop.is_set():
#                 break
#             if ev.is_set():
#                 goal = self._goals_ticks[sid]
#                 ev.clear()
#                 if goal != last_sent:
#                     with self._comm_lock:
#                         # send_goal(sid, ticks, speed, acc)
#                         self._pkt.send_goal(sid, int(goal), self.speed_default, self.acc_default)
#                     last_sent = goal

#     def _safe_read_abs_pos(self, sid: int) -> int:
#         with self._comm_lock:
#             val = self._pkt.ReadAbsPos(sid)
#         return 0 if val is None else int(val)

#     # -------------- unit conversions --------------
#     def _sid_from_joint(self, joint_name: str) -> int:
#         try:
#             idx = self.joint_order.index(joint_name)
#         except ValueError:
#             raise KeyError(f"Unknown joint name '{joint_name}'. Check real_config.joint_order.")
#         return self.motor_ids[idx]

#     def _rad_to_ticks(self, sid: int, angle_rad: float) -> float:
#         """
#         ticks = zero + angle_rad * (ticks_per_turn / 2π) * gear_ratio
#         """
#         gr = float(self.gear_ratio.get(sid, 1.0))
#         zero = float(self.zero_off_rad.get(sid, 0.0))
#         mech = angle_rad - zero
#         return mech * (self.ticks_per_turn / _TWO_PI) * gr

#     def _ticks_to_rad(self, sid: int, ticks: int) -> float:
#         gr = float(self.gear_ratio.get(sid, 1.0))
#         zero = float(self.zero_off_rad.get(sid, 0.0))
#         mech = float(ticks) / (self.ticks_per_turn * gr) * _TWO_PI
#         return mech + zero
#     def _deg_to_ticks(self, sid: int, offset_deg: float) -> float:
#         """
#         Convert the zero offset (in degrees) to encoder ticks for the given motor.
#         """
#         gr = float(self.gear_ratio.get(sid, 1.0))  # Gear ratio for the motor
#         ticks = offset_deg * (self.ticks_per_turn / 360.0) * gr  # Convert from degrees to ticks
#         return ticks
