# servopkg/highlevel.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Iterable, Optional
import math
import threading
import time

from .bus import ServoBus

_TWO_PI = 2.0 * math.pi

@dataclass
class ServoKinematics:
    ticks_per_turn: int = 4096
    gear_ratio: float   = 1.0     # motor_turns per joint_turn
    orientation: int    = 1       # +1 or -1 (flip)
    pos_offset_rad: float = 0.0   # extra mechanical offset (applied in radians)
    home_ticks: int       = 0     # device absolute tick at home
    home_loops: int       = 0     # loop count at home (for "infinite" ticks)

class MultiServoController:
    """
    Owns:
      - SDK access (through ServoBus)
      - per-servo config/state
      - rad <-> total ticks conversions
      - loop counting (unwrap device 0..4095 to infinite ticks)
      - worker threads to send goals

    Exposes:
      - read_angles_rad(joint_ids)
      - set_goal_angle_rad(sid, angle_rad)
      - set_group_goals_rad({sid: angle_rad})
      - shutdown()
    """
    def __init__(
        self,
        bus: ServoBus,
        servo_ids: List[int],
        kin: Dict[int, ServoKinematics],
        speed_default: int = 133,
        acc_default: int = 50,
        speed_min: int = 1,
        speed_max: int = 4095,
        init_position_mode: bool = True,
        disable_limits: bool = True,
    ):
        self._bus = bus
        self._pkt = bus.sdk

        self._servo_ids = list(servo_ids)
        self._kin: Dict[int, ServoKinematics] = {int(k): v for k, v in kin.items()}
        self._speed_default = int(speed_default)
        self._acc_default   = int(acc_default)
        self._speed_min     = int(speed_min)
        self._speed_max     = int(speed_max)

        # Runtime state
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._events: Dict[int, threading.Event] = {}
        self._goals_total_ticks: Dict[int, int] = {}

        # For "infinite" tick tracking
        self._prev_ticks: Dict[int, int] = {}
        self._loop_count: Dict[int, int] = {}

        # Init servos and state
        for sid in self._servo_ids:
            if init_position_mode:
                self._pkt.ChangeMode(sid, 0)  # 0 -> position mode
            if disable_limits:
                self._pkt.ChangeMaxLimit(sid, 0)
                self._pkt.ChangeMinLimit(sid, 0)

            cur = self._safe_read_abs_pos(sid)
            self._goals_total_ticks[sid] = cur
            self._prev_ticks[sid] = cur
            self._loop_count[sid] = self._kin.get(sid, ServoKinematics()).home_loops

            self._events[sid] = threading.Event()

        # Start per-servo workers
        self._workers: List[threading.Thread] = []
        for sid in self._servo_ids:
            t = threading.Thread(target=self._worker, args=(sid,), daemon=True, name=f"servo-{sid}")
            t.start()
            self._workers.append(t)

    # ---------- Public API ----------

    def read_angles_rad(self, sids: Iterable[int]) -> Dict[int, float]:
        """
        Returns joint angles in radians for each sid requested.
        Uses infinite tick tracking and converts total ticks -> radians.
        """
        out: Dict[int, float] = {}
        for sid in sids:
            ticks = self._safe_read_abs_pos(sid)
            k = self._kin.get(sid, ServoKinematics())
            half = k.ticks_per_turn // 2

            # update loop count
            raw = ticks - self._prev_ticks[sid]
            if   raw >  half: self._loop_count[sid] -= 1  # wrapped backwards (4095->0)
            elif raw < -half: self._loop_count[sid] += 1  # wrapped forwards (0->4095)
            self._prev_ticks[sid] = ticks

            total = self._loop_count[sid] * k.ticks_per_turn + ticks
            home_total = k.home_loops * k.ticks_per_turn + k.home_ticks

            # Convert to radians at the JOINT side (divide by gear)
            joint_mech_ticks = (total - home_total) / max(k.gear_ratio, 1e-9)
            angle = (joint_mech_ticks / k.ticks_per_turn) * _TWO_PI
            angle *= k.orientation  # orientation finally
            angle -= k.pos_offset_rad
            out[sid] = angle
        return out

    def set_goal_angle_rad(self, sid: int, angle_rad: float):
        goal_total = self._angle_rad_to_total_ticks(sid, angle_rad)
        with self._lock:
            self._goals_total_ticks[sid] = int(round(goal_total))
            self._events[sid].set()

    def set_group_goals_rad(self, goals: Dict[int, float]):
        with self._lock:
            for sid, angle_rad in goals.items():
                self._goals_total_ticks[sid] = int(round(self._angle_rad_to_total_ticks(sid, angle_rad)))
                self._events[sid].set()

    def shutdown(self):
        self._stop.set()
        for ev in self._events.values():
            ev.set()
        time.sleep(0.05)
        self._bus.close()

    # ---------- Internals ----------

    def _safe_read_abs_pos(self, sid: int) -> int:
        with self._lock:
            val = self._pkt.ReadAbsPos(sid)
        return 0 if val is None else int(val)

    def _angle_rad_to_total_ticks(self, sid: int, angle_rad: float) -> float:
        k = self._kin.get(sid, ServoKinematics())
        # apply offsets/orientation at JOINT side
        joint_angle = (angle_rad + k.pos_offset_rad) * k.orientation

        # Convert to motor ticks (gear * ticks_per_turn / 2π)
        motor_mech_ticks = joint_angle * (k.ticks_per_turn / _TWO_PI) * k.gear_ratio

        home_total = k.home_loops * k.ticks_per_turn + k.home_ticks
        return home_total + motor_mech_ticks

    def _speed_for(self, sid: int) -> int:
        k = self._kin.get(sid, ServoKinematics())
        # You can scale by gear if you want; here we simply clamp defaults:
        spd = int(self._speed_default)
        return max(self._speed_min, min(self._speed_max, spd))

    def _worker(self, sid: int):
        last_sent = None
        while not self._stop.is_set():
            ev = self._events[sid]
            ev.wait(timeout=0.25)
            if self._stop.is_set(): break
            if not ev.is_set():     continue

            with self._lock:
                goal_total = self._goals_total_ticks[sid]
                speed      = self._speed_for(sid)
                ev.clear()

            # Send only if changed
            if goal_total != last_sent:
                # NOTE: send_goal expects "total ticks" if your SDK supports multi-turn,
                # otherwise your SDK might modulo internally. Keep it here for consistency.
                self._pkt.send_goal(sid, int(goal_total), speed, self._acc_default)
                last_sent = goal_total
