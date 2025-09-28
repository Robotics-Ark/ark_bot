
import time
import os
import numpy as np
import pickle
from arktypes import joint_group_command_t, task_space_command_t, joint_state_t
from arktypes.utils import pack, unpack
from ark.client.comm_infrastructure.instance_node import InstanceNode

ROBOT_NAME = "arkbot"
SIM = False

class HarkControllerNode(InstanceNode):
    def __init__(self):
        super().__init__("arkbot_controller")
        suffix = "/sim" if SIM else ""
        base = f"{ROBOT_NAME}"

        # Publisher to send joint group commands
        self.joint_group_command = self.create_publisher(f"{base}/joint_group_command{suffix}", joint_group_command_t)
        self.task_space_command = self.create_publisher(f"{base}/cartesian_command{suffix}", task_space_command_t)

        # Subscriber to receive joint states (if needed for feedback)
        self.state = self.create_listener(f"{base}/joint_states{suffix}", joint_state_t)

# Initialize the node for controlling the robot
node = HarkControllerNode()

# Define the different waypoints
home =  [0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0]
approach_upper = [0.4, 0.3, -1.2, 2.0, 0.0, 2.2, 0.0]
contact_upper = [0.4, 0.3, -1.2, 1.35, 0.0, 1.4, 0.0]
lower_upper = [0.4, 0.3, -1.2, 2.0, 0.0, 2.2, 0.0]
approach_lower = [0.4, 0.3, -1.8, 2.0, 0.0, 2.2, 0.0]
contact_lower = [0.4, 0.4, -1.8, 1.5, 0.0, 2.2, 0.0]
retract_lower = [0.4, -0.3, -1.8, 2.0, 0.0, 2.2, 0.0]
push_lower = [0.4, 0.4, -1.8, 0.6, 0.0, 1.5, 0.0]

# Create save directory for joint states
save_dir = "joint_state_data"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

def get_joint_positions():
    data = unpack.joint_state(node.state.get())  # Capture the full joint state data
    return data[2]

# Function to capture the joint state periodically and save it
def capture_joint_state(filename, rate=1.0):
    joint_states = []
    start_time = time.time()

    while True:
        # Capture the current joint state
        joint_state = unpack.joint_state(node.state.get())
        joint_states.append(joint_state)

        # Save the joint state to file every second (or at the specified rate)
        if time.time() - start_time >= rate:
            with open(filename, "wb") as f:
                pickle.dump(joint_states, f)
            start_time = time.time()

        # Break out after a specific number of movements (for demo purposes)
        if len(joint_states) >= 8:  # Assuming we're making 8 movements
            break
        
        time.sleep(rate)  # Capture at the defined rate

# Function to move robot through waypoints and capture joint states
def move_robot_and_capture_state():
        # Move through waypoints and capture joint states
    node.joint_group_command.publish(pack.joint_group_command(home, "all"))
    time.sleep(1)
    node.joint_group_command.publish(pack.joint_group_command(approach_upper, "all"))
    time.sleep(6)
    node.joint_group_command.publish(pack.joint_group_command(contact_upper, "all"))
    time.sleep(1)
    node.joint_group_command.publish(pack.joint_group_command(lower_upper, "all"))
    time.sleep(1)
    node.joint_group_command.publish(pack.joint_group_command(approach_lower, "all"))
    time.sleep(3)
    node.joint_group_command.publish(pack.joint_group_command(contact_lower, "all"))
    time.sleep(1)
    node.joint_group_command.publish(pack.joint_group_command(push_lower, "all"))
    time.sleep(1)
    node.joint_group_command.publish(pack.joint_group_command(retract_lower, "all"))
    time.sleep(4)
    node.joint_group_command.publish(pack.joint_group_command(home, "all"))

# Start the joint state capture in a separate thread
import threading

# Set filename and capture rate (every 1 second in this case)
filename = os.path.join(save_dir, f"joint_states_{time.time():.0f}.pkl")  # Use a timestamp in the filename
capture_rate = 1.0  # capture joint state every second

capture_thread = threading.Thread(target=capture_joint_state, args=(filename, capture_rate))
capture_thread.start()

# Move the robot and capture joint states
move_robot_and_capture_state()

# Wait for the capture thread to finish
capture_thread.join()

print(f"Robot movement completed and joint states saved to {filename}.")
