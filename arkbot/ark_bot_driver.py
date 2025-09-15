# ark_bot_driver.py
from typing import Dict, Any, List
import math
import time
import threading

from ark.system.driver.robot_driver import RobotDriver
from ark.tools.log import log

# ---- Your servo SDK ----
from servopkg import PortHandler, sts  # expects .ReadAbsPos, ChangeMode, send_goal, ...

_TWO_PI = 2.0 * math.pi

class ArkBotDriver(RobotDriver):
    """
    ARK driver that talks to your the ark bot motors using servopkg.

    Notes:
      - This driver publishes/accepts joint angles in *radians*.
      - It convert radians <-> absolute encoder ticks via gear_ratio & zero_offset.

    """

    def __init__(self, component_name: str, component_config: Dict[str, Any] = None, sim: bool = False):
        super().__init__(component_name, component_config, sim)
        
        rc = self.config["real_config"]

        # Serial
        self.port = rc["port"]
        self.baud = int(rc.get("baudrate", 1_000_000))

        # Joint mapping
        self.joint_order: List[str] = list(rc["joint_order"])        # ordered joint names
        self.motor_ids: List[int] = [int(x) for x in rc["motor_ids"]]# same length/order as joint_order
        assert len(self.joint_order) == len(self.motor_ids), "joint_order and motor_ids length mismatch"

        # Kinematic conversion
        self.ticks_per_turn = float(rc.get("ticks_per_turn", 4096))
        gr = rc.get("gear_ratios", {})
        zo = rc.get("zero_offsets_deg", {})
        # normalize keys to int
        self.gear_ratio: Dict[int, float] = {int(k): float(v) for k, v in gr.items()}
        self.zero_off_rad: Dict[int, float] = {int(k): math.radians(float(v)) for k, v in zo.items()}

        # Motion defaults
        self.speed_default = int(rc.get("speed_default", 1200))
        self.acc_default   = int(rc.get("acc_default", 50))

        # SDK + threading
        self._port = PortHandler(self.port)
        if not self._port.openPort():
            raise RuntimeError(f"Failed to open port {self.port}")
        if not self._port.setBaudRate(self.baud):
            raise RuntimeError(f"Failed to set baudrate {self.baud}")

        self._pkt = sts(self._port)

        # Protect serial access
        self._comm_lock = threading.Lock()

        # Current goals in ticks (absolute), per motor-id
        self._goals_ticks: Dict[int, int] = {}
        # Per-motor event to wake worker
        self._events: Dict[int, threading.Event] = {}
        # Stop flag
        self._stop = threading.Event()

        # Configure motors and seed goals with current abs pos
        for sid in self.motor_ids:
            with self._comm_lock:
                self._pkt.ChangeMode(sid, 0)       # position mode
                self._pkt.ChangeMaxLimit(sid, 0)   # disable lib-side limits if 0 means no limit
                self._pkt.ChangeMinLimit(sid, 0)
            cur_ticks = self._safe_read_abs_pos(sid)
            self._goals_ticks[sid] = cur_ticks
            self._events[sid] = threading.Event()

        # Workers
        self._workers: List[threading.Thread] = []
        for sid in self.motor_ids:
            t = threading.Thread(target=self._motor_worker, args=(sid,), daemon=True, name=f"motor-{sid}")
            t.start()
            self._workers.append(t)

        log.info(f"[{component_name}] ArkBotDriver initialised on {self.port} @ {self.baud}")
    
    # ======================
    # Core Functions
    # ======================

    def shutdown_driver(self):
        # called by ARK when stopping
        self._stop.set()
        for ev in self._events.values():
            ev.set()
        time.sleep(0.1)
        try:
            self._port.closePort()
        except Exception:
            pass
        log.info("ArkBotDriver shutdown complete")

    # ======================
    # Driver Functions
    # ======================
    
    def pass_joint_positions(self, joints: List[str]) -> Dict[str, float]:
        """
        Return {joint_name: angle_in_radians} for the requested joints.
        """
        out: Dict[str, float] = {}
        for jname in joints:
            sid = self._sid_from_joint(jname)
            ticks = self._safe_read_abs_pos(sid)
            ang = self._ticks_to_rad(sid, ticks)
            out[jname] = ang
        return out

    def pass_joint_velocities(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def pass_joint_efforts(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    def check_torque_status(self, joints: List[str]) -> Dict[str, float]:
        raise NotImplementedError

    # ======================
    # Control Functions
    # ======================

    def pass_joint_group_control_cmd(self, control_mode: str, cmd: Dict[str, float], **kwargs) -> None:

        group = kwargs.get("group_name", "arm")
        print(f"Received joint group command for group '{group}': {cmd}")
        if control_mode == "position":
            # Convert each joint target rad -> ticks and queue for its worker
            for jname in cmd.keys():
                sid = self._sid_from_joint(jname)
                target_rad = float(cmd[jname])
                target_ticks = self._rad_to_ticks(sid, target_rad)
                # update goal and wake worker
                self._goals_ticks[sid] = int(target_ticks)
                self._events[sid].set()

        elif control_mode == "velocity":
            log.warn(f"Velocity mode not implemented; ignoring command for group {group}")
        else:
            log.error(f"Unsupported control_mode: {control_mode}")

        time.sleep(0.001)  # tiny yield

    def pass_cartesian_control_cmd(self, control_mode, position, quaternion, **kwargs) -> None:
        log.warn("Cartesian control not implemented in ArkBotDriver; ignoring.")

    # -------------- workers & IO helpers --------------
    def _motor_worker(self, sid: int):
        last_sent = None
        while not self._stop.is_set():
            ev = self._events[sid]
            ev.wait(timeout=0.25)
            if self._stop.is_set():
                break
            if ev.is_set():
                goal = self._goals_ticks[sid]
                ev.clear()
                if goal != last_sent:
                    with self._comm_lock:
                        # send_goal(sid, ticks, speed, acc)
                        self._pkt.send_goal(sid, int(goal), self.speed_default, self.acc_default)
                    last_sent = goal

    def _safe_read_abs_pos(self, sid: int) -> int:
        with self._comm_lock:
            val = self._pkt.ReadAbsPos(sid)
        return 0 if val is None else int(val)

    # -------------- unit conversions --------------
    def _sid_from_joint(self, joint_name: str) -> int:
        try:
            idx = self.joint_order.index(joint_name)
        except ValueError:
            raise KeyError(f"Unknown joint name '{joint_name}'. Check real_config.joint_order.")
        return self.motor_ids[idx]

    def _rad_to_ticks(self, sid: int, angle_rad: float) -> float:
        """
        ticks = zero + angle_rad * (ticks_per_turn / 2π) * gear_ratio
        """
        gr = float(self.gear_ratio.get(sid, 1.0))
        zero = float(self.zero_off_rad.get(sid, 0.0))
        mech = angle_rad - zero
        return mech * (self.ticks_per_turn / _TWO_PI) * gr

    def _ticks_to_rad(self, sid: int, ticks: int) -> float:
        gr = float(self.gear_ratio.get(sid, 1.0))
        zero = float(self.zero_off_rad.get(sid, 0.0))
        mech = float(ticks) / (self.ticks_per_turn * gr) * _TWO_PI
        return mech + zero
