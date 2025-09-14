# my_robot.py
from typing import Any, Dict
from dataclasses import dataclass
from enum import Enum
import numpy as np

from ark.client.comm_infrastructure.base_node import main
from ark.system.component.robot import Robot
from ark.system.driver.robot_driver import RobotDriver
from ark.system.pybullet.pybullet_robot_driver import BulletRobotDriver
from ark.tools.log import log
from arktypes import joint_state_t, joint_group_command_t, task_space_command_t
from arktypes.utils import unpack



@dataclass
class Drivers(Enum):
    PYBULLET_DRIVER = BulletRobotDriver
    try:
        from ark_bot_driver import MyRobotDriver  
        DRIVER = MyRobotDriver
    except ImportError:
        log.warn("MyRobotDriver is failing")
    

class MyRobot(Robot):
    def __init__(self, name: str, global_config: Dict[str, Any] = None, driver: RobotDriver = None):
        super().__init__(name=name, global_config=global_config, driver=driver)
    
        self.joint_group_command_ch = f"{self.name}/joint_group_command"
        self.cartesian_position_control_ch = f"{self.name}/cartesian_command"
        if self.sim:
            self.joint_group_command_ch += "/sim"
            self.cartesian_position_control_ch += "/sim"
       
        self.create_subscriber(self.joint_group_command_ch, joint_group_command_t, self._joint_group_command_cb)
        self.create_subscriber(self.cartesian_position_control_ch, task_space_command_t, self._cartesian_position_cb)
        
        self.joint_states_pub = f"{self.name}/joint_states" + ("/sim" if self.sim else "")
        self.component_channels_init({ self.joint_states_pub: joint_state_t })

        self.joint_group_command = None
        self.cartesian_position_control_command = None

    def control_robot(self):
        if self.joint_group_command:
            group_name = self.joint_group_command["name"]
            ordered_names = list(self.joint_groups[group_name]["joints"])
            ordered_vals = self.joint_group_command["cmd"]
            cmd_dict = dict(zip(ordered_names, ordered_vals))
            control_mode = self.joint_groups[group_name]["control_mode"]
            self.control_joint_group(control_mode, cmd_dict)
            self.joint_group_command = None

        if self.cartesian_position_control_command:
            group_name = self.cartesian_position_control_command["name"]
            control_mode = self.joint_groups[group_name]["control_mode"]
            ee_idx = self.config.get("ee_index", 5) 
            self._driver.pass_cartesian_control_cmd(
                control_mode,
                position=self.cartesian_position_control_command["position"],
                quaternion=self.cartesian_position_control_command["quaternion"],
                end_effector_idx=ee_idx,
                gripper=self.cartesian_position_control_command.get("gripper", None),
            )
            self.cartesian_position_control_command = None

    def get_state(self) -> Dict[str, Any]:
        joints = self.get_joint_positions()
        return {"joint_positions": joints}

    def pack_data(self, state: Dict[str, Any]) -> Dict[str, Any]:
        joint_state = state["joint_positions"]

        msg = joint_state_t()
        msg.n = len(joint_state)
        msg.name = list(joint_state.keys())
        msg.position = list(joint_state.values())
        msg.velocity = [0.0] * msg.n
        msg.effort   = [0.0] * msg.n

        return { self.joint_states_pub: msg }

    def _joint_group_command_cb(self, t, ch, msg):
        cmd, name = unpack.joint_group_command(msg)
        self.joint_group_command = {"cmd": cmd, "name": name}

    def _cartesian_position_cb(self, t, ch, msg):
        name, position, quaternion, gripper = unpack.task_space_command(msg)
        self.cartesian_position_control_command = {
            "name": name, "position": position, "quaternion": quaternion, "gripper": gripper
        }

CONFIG_PATH = "arkbot.yaml"
if __name__ == "__main__":
    name = "Arkbot"
    driver = MyRobotDriver(name, CONFIG_PATH, sim=False)
    main(MyRobot, name, CONFIG_PATH, driver)

