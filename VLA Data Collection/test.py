import time
import os
import pickle
import numpy as np
from arktypes import joint_group_command_t, task_space_command_t, joint_state_t
from arktypes.utils import pack, unpack
from ark.client.comm_infrastructure.instance_node import InstanceNode
import threading

# Assuming DiffusionEnv is imported or defined elsewhere
# from diffusion_env import DiffusionEnv

ROBOT_NAME = "arkbot"
SIM = False
CONFIG = "../config/global_config.yaml"  # Path to your configuration file
SAVE_DIR = "drawer_push_trajectories"
os.makedirs(SAVE_DIR, exist_ok=True)

# Robot Controller Node for communication with robot
class HarkControllerNode(InstanceNode):
    def __init__(self):
        super().__init__("arkbot_controller")
        suffix = "/sim" if SIM else ""
        base = f"{ROBOT_NAME}"

        # Publisher for joint commands
        self.joint_group_command = self.create_publisher(f"{base}/joint_group_command{suffix}", joint_group_command_t)
        
        # Subscriber for joint states
        self.state = self.create_listener(f"{base}/joint_states{suffix}", joint_state_t)

node = HarkControllerNode()
time.sleep(1)  # Wait for connections to establish

# Define the waypoints (same as before)
waypoints = [
    [0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0],
    [0.4, 0, -1.2, 2.0, 0.0, 2.2, 0.0],
    [0.4, 0.3, -1.2, 2.0, 0.0, 2.2, 0.0],
    [0.4, 0.3, -1.2, 1.35, 0.0, 1.4, 0.0],
    [0.4, 0.3, -1.2, 2.0, 0.0, 2.2, 0.0],
    [0.4, 0.3, -1.8, 2.0, 0.0, 2.2, 0.0],
    [0.4, 0.4, -1.8, 1.5, 0.0, 2.2, 0.0],
    [0.4, -0.3, -1.8, 2.0, 0.0, 2.2, 0.0],
    [0.4, 0.4, -1.8, 0.6, 0.0, 1.5, 0.0]
]

# Interpolated trajectory based on waypoints
class DrawerPushPolicy:
    def __init__(self, steps_per_phase=100):
        self.state_id = 0
        self.steps_per_phase = steps_per_phase
        self.trajectory = []

    def plan_trajectory(self):
        self.trajectory = []
        for i in range(len(waypoints) - 1):
            start = np.array(waypoints[i])
            end = np.array(waypoints[i + 1])
            for s in range(self.steps_per_phase):
                alpha = s / self.steps_per_phase
                joints = (1 - alpha) * start + alpha * end
                self.trajectory.append(joints.tolist())

    def __call__(self):
        if not self.trajectory:
            self.plan_trajectory()

        if self.state_id >= len(self.trajectory):
            joints = self.trajectory[-1]  # hold last pose
        else:
            joints = self.trajectory[self.state_id]
            self.state_id += 1
        print(joints)
        return joints

# Simulation environment for the robot
class DiffusionEnv:
    def __init__(self, config, sim=True):
        # Here you'd initialize your environment using the config file
        self.config = config
        self.sim = sim
        self.state = None
        self.reset()

    def reset(self):
        # Reset the environment and return initial observation and info
        self.state = {"position": np.zeros(7)}  # Example state with 7 DoF joints
        return self.state, {}

    def step(self, action):
        # Step the environment using the action (robot joint command)
        next_state = {"position": np.array(action)}  # Update state based on action
        reward = 0.0  # Placeholder reward
        terminated = False  # Define termination condition
        truncated = False  # Define truncation condition
        info = {}
        return next_state, reward, terminated, truncated, info

# Initialize the Diffusion environment
env = DiffusionEnv(config=CONFIG, sim=True)

# Start the controller node
policy = DrawerPushPolicy(steps_per_phase=75)

# Function to capture joint states periodically
def capture_joint_state(filename, rate=1.0, done_event=None):
    joint_states = []
    start_time = time.time()

    while not done_event.is_set():  # Wait for the 'done_event' to be set
        joint_state_data = unpack.joint_state(node.state.get())  # Capture the current joint state
        joint_positions = joint_state_data[2]  # Extract joint positions
        print(joint_positions)
        joint_states.append(joint_positions)

        if time.time() - start_time >= rate:
            try:
                with open(filename, "wb") as f:
                    pickle.dump(joint_states, f)
                print(f"Joint states saved to {filename}.")
            except Exception as e:
                print(f"Error saving joint states: {e}")
            
            start_time = time.time()
        
        time.sleep(rate)

    print("Joint state capture complete.")

# Runner for moving robot and collecting data
def move_robot_and_collect_data(num_trajectories=300):
    for traj_id in range(num_trajectories):
        print(f"Starting trajectory {traj_id + 1}/{num_trajectories}...")

        # Create a filename for the joint state data
        filename = os.path.join(SAVE_DIR, f"trajectory_{traj_id:03d}.pkl")

        # Create event to stop joint state capture thread
        done_event = threading.Event()

        # Start joint state capture in a separate thread
        capture_thread = threading.Thread(target=capture_joint_state, args=(filename, 0.1, done_event))
        capture_thread.start()

        # Reset environment or robot (here we simulate with sleep)
        observation, _ = env.reset()  # Reset environment to get initial observation
        time.sleep(2.0)  # Wait for reset to complete

        # Move through the planned trajectory
        for joint_cmd in policy():
            node.joint_group_command.publish(pack.joint_group_command(joint_cmd, "all"))
            observation, reward, terminated, truncated, _ = env.step(joint_cmd)
            time.sleep(0.1)  # small delay to simulate robot movement
            print(f"Moving to joint positions: {joint_cmd}")

        # Once the movement is completed, stop joint state capture
        done_event.set()
        capture_thread.join()

        print(f"Trajectory {traj_id} complete. Joint states saved to {filename}.")

# Run the robot movements and data collection
move_robot_and_collect_data(num_trajectories=300)
