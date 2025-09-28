import os
import time
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R


class DrawerPushPolicy:
    def __init__(self, steps_per_phase=100):
        self.state_id = 0
        self.steps_per_phase = steps_per_phase
        self.trajectory = []

    def plan_trajectory(self, obs):
        """
        Drawer push trajectory consists of joint waypoints.
        We'll interpolate between them.
        obs can provide robot base state or be ignored here.
        """

        # Define waypoints (7-DoF joints)
        waypoints = [
            [0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0],  # home
            [0.0, 0.0, 0.0, 2.60, 0.0, -0.9, 0.0],  # approach_upper
            [0.0, 0.4, 0.0, 2.00, 0.0, -0.9, 0.0],  # contact_upper
            [0.0, 0.8, 0.0, 1.50, 0.0, -0.9, 0.0],  # push_upper
            [0.0, 0.0, 0.0, 2.60, 0.0, -0.9, 0.0],  # retract
            [0.0, 0.6, 0.0, 2.30, 0.0, -0.9, 0.0],  # approach_lower
            [0.0, 0.8, 0.0, 2.00, 0.0, -1.2, 0.0],  # contact_lower
            [0.0, 1.0, 0.0, 1.50, 0.0, -1.2, 0.0],  # push_lower
            [0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0],  # retract_home
        ]

        # Interpolated trajectory
        self.trajectory = []
        for i in range(len(waypoints) - 1):
            start = np.array(waypoints[i])
            end = np.array(waypoints[i + 1])
            for s in range(self.steps_per_phase):
                alpha = s / self.steps_per_phase
                joints = (1 - alpha) * start + alpha * end
                self.trajectory.append(joints.tolist())

    def __call__(self, obs):
        if not self.trajectory:
            self.plan_trajectory(obs)

        if self.state_id >= len(self.trajectory):
            # hold last pose
            joints = self.trajectory[-1]
        else:
            joints = self.trajectory[self.state_id]
            self.state_id += 1

        return joints


# ----------------- Runner for data collection ----------------- #

CONFIG = "../config/global_config.yaml"
num_trajectories = 300
save_dir = "drawer_push_trajectories"
os.makedirs(save_dir, exist_ok=True)

env = DiffusionEnv(config=CONFIG, sim=True)  # same as your setup

for traj_id in range(num_trajectories):
    observation, info = env.reset()
    policy = DrawerPushPolicy(steps_per_phase=75)
    time.sleep(2.0)  # allow environment to reset
    trajectory = []

    while True:
        action = policy(observation)  # joint command (7 values)
        next_observation, reward, terminated, truncated, info = env.step(action)

        # Flatten state + action (adapt as needed to your observation structure)
        state = (
            list(observation["position"])  # robot joint states
            # + list(observation["franka_ee"])  # end effector pose
            + list(observation["images"])  # if you log images
        )

        row = {"state": state, "action": list(action)}
        trajectory.append(row)

        if terminated:
            print(f"SUCCESS: Trajectory {traj_id} ended. Saving...")
            pkl_path = os.path.join(save_dir, f"trajectory_{traj_id:03d}.pkl")
            with open(pkl_path, "wb") as f:
                pickle.dump(trajectory, f)
            break
        elif truncated:
            print(f"FAILED: Trajectory {traj_id} truncated.")
            break
        else:
            observation = next_observation

        time.sleep(0.05)  # small step delay
