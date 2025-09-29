from ark.env.ark_env import ArkEnv
from arktypes import joint_group_command_t, joint_state_t, rigid_body_state_t
from arktypes.utils import pack, unpack
from arktypes import task_space_command_t, pose_t
import numpy as np


class DiffusionEnv(ArkEnv):

    def __init__(self, config=None, sim=True):
        environment_name = "diffusion_env"

        action_channels = {
            "arkbot/cartesian_command/sim": task_space_command_t,
        }

        observation_channels = {
            "arkbot/ee_state/sim": pose_t,
            "arkbot/joint_states/sim": joint_state_t,
        }

        self.steps = 0
        self.max_steps = 500

        super().__init__(
            environment_name=environment_name,
            action_channels=action_channels,
            observation_channels=observation_channels,
            global_config=config,
            sim=sim,
        )

    def action_packing(self, action):
        """
        Packs the action into a joint_group_command_t format.

        List of:
        [EE X, EE_Y, EE_Z, EE_Quaternion_X, EE_Quaternion_Y, EE_Quaternion_Z, EE_Quaternion_W, Gripper]
        """

        xyz_command = np.array(action[:3])
        quaternion_command = np.array(action[3:7])
        gripper_command = action[7]

        arkbot_cartesian_command = pack.task_space_command(
            "all", xyz_command, quaternion_command, gripper_command
        )
        return {"arkbot/cartesian_command/sim": arkbot_cartesian_command}

    def observation_unpacking(self, observation_dict):
        """
        Unpacks the observation from the environment.

        Returns a dictionary with keys
        """
        joint_state = observation_dict["arkbot/joint_states/sim"]
        ee_state = observation_dict["arkbot/ee_state/sim"]


        _, _, arkbot_joint_position, _, _ = unpack.joint_state(joint_state)

        gripper_position = arkbot_joint_position[
            -2
        ]  # Assuming last two joints are gripper

        return {
            "cube": cube_position,
            "target": target_position,
            "gripper": [gripper_position],
        }

    def reset_objects(self):
        self.steps = 0
        self.reset_component("cube")
        self.reset_component("target")
        self.reset_component("arkbot")

    def reward(self, state, action, next_state):
        return 0.0

    def step(self, action):
        self.steps += 1
        return super().step(action)

    def terminated_truncated_info(self, state, action, next_state):
        cube_pos = np.array(state["cube"])
        target_pos = np.array(state["target"])

        # Terminate if cube is within 5 cm of target
        distance = np.linalg.norm(cube_pos - target_pos)
        terminated = distance < 0.1

        if terminated:
            print("Cube is close enough to the target. Terminating episode.")

        if self.steps >= self.max_steps:
            print("Max steps reached. Terminating episode.")
            truncated = True
        else:
            truncated = False

        return terminated, truncated, self.steps