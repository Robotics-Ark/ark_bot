import time
import os
import numpy as np
import pickle
from arktypes import joint_group_command_t, task_space_command_t, joint_state_t
from arktypes.utils import pack, unpack
from ark.client.comm_infrastructure.instance_node import InstanceNode
import threading

ROBOT_NAME = "arkbot"
SIM = False

class HarkControllerNode(InstanceNode):
    def __init__(self):
        super().__init__("arkbot_controller")
        suffix = "/sim" if SIM else ""
        base = f"{ROBOT_NAME}"

        # pubs
        self.joint_group_command = self.create_publisher(f"{base}/joint_group_command{suffix}", joint_group_command_t)
        self.task_space_command = self.create_publisher(f"{base}/cartesian_command{suffix}", task_space_command_t)

        # subs
        self.state = self.create_listener(f"{base}/joint_states{suffix}", joint_state_t)

node = HarkControllerNode()
time.sleep(1)  # wait for connections to establish

# Define the different waypoints
home =  [0.0, 0.0, 0.0, 0.00, 0.0, 0.0, 0.0]
approach_upper = [0.4, 0, 1.2, -2.0, 0.0, 2, 0.0]
contact_upper = [0.4, -0.3, 1.3, -2.0, 0.0, 2.2, 0.0]
push_upper = [0.4, -0.3, 1.3, -1.2, 0.0, 1.6, 0.0]
lower_upper = [0.4, -0.3, 1.3, -2.0, 0.0, 2.2, 0.0]
approach_lower = [0.4, -0.3, 1.8, -2.0, 0.0, 2.2, 0.0]
contact_lower = [0.4, -0.4, 1.8, -1.5, 0.0, 2.2, 0.0]
push_lower = [0.4, -0.4, 1.8, -0.5, 0.0, 1.4, 0.0]
retract_lower = [0.4, 0.3, 1.8, -2.0, 0.0, 2.2, 0.0]

# Create save directory for joint states
save_dir = "joint_state_data"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

def get_joint_positions():
    data = unpack.joint_state(node.state.get())  # Capture the full joint state data
    return data[2]

# Function to capture the joint state periodically and save it
def capture_joint_state(filename, rate=1.0, done_event=None):
    joint_states = []
    start_time = time.time()

    while not done_event.is_set():  # Wait for the 'done_event' to be set
        # Capture the current joint state
        joint_state_data = unpack.joint_state(node.state.get())
        joint_positions = joint_state_data[2]
        print(joint_positions)
        joint_states.append(joint_positions)

        # Debugging: Print the time difference and rate comparison
        print(f"Time elapsed: {time.time() - start_time:.2f} s, Rate: {rate} s")

        # Save the joint state to file every second (or at the specified rate)
        if time.time() - start_time >= rate:
            try:
                with open(filename, "wb") as f:
                    pickle.dump(joint_states, f)
                print(f"Joint states saved to {filename}.")
            except Exception as e:
                print(f"Error saving joint states: {e}")
            
            # Reset start_time after saving
            start_time = time.time()
        
        time.sleep(rate)  # Sleep for the capture rate

    print("Joint state capture complete.")

# Function to move robot through waypoints and capture joint states
def move_robot_and_capture_state():
    # Move through waypoints and capture joint states
    node.joint_group_command.publish(pack.joint_group_command(approach_upper, "all"))
    time.sleep(1)
    node.joint_group_command.publish(pack.joint_group_command(approach_upper, "all"))
    time.sleep(6)
    node.joint_group_command.publish(pack.joint_group_command(contact_upper, "all"))
    time.sleep(2)
    node.joint_group_command.publish(pack.joint_group_command(push_upper, "all"))
    time.sleep(3)
    node.joint_group_command.publish(pack.joint_group_command(lower_upper, "all"))
    time.sleep(1)
    node.joint_group_command.publish(pack.joint_group_command(approach_lower, "all"))
    time.sleep(3)
    node.joint_group_command.publish(pack.joint_group_command(contact_lower, "all"))
    time.sleep(2)
    node.joint_group_command.publish(pack.joint_group_command(push_lower, "all"))
    time.sleep(4)
    node.joint_group_command.publish(pack.joint_group_command(contact_lower, "all"))
    time.sleep(1)
    node.joint_group_command.publish(pack.joint_group_command(retract_lower, "all"))
    time.sleep(4)
    node.joint_group_command.publish(pack.joint_group_command(approach_upper, "all"))

# Create an event for signaling when capture is done
done_event = threading.Event()

# Set filename and capture rate (every 0.1 second in this case)
filename = os.path.join(save_dir, f"lower_joint_states_{time.time():.0f}.pkl")  # Use a timestamp in the filename
capture_rate = 0.1  # capture joint state every 0.1 second

# Start the joint state capture in a separate thread
capture_thread = threading.Thread(target=capture_joint_state, args=(filename, capture_rate, done_event))
capture_thread.start()

# Move the robot and capture joint states
move_robot_and_capture_state()

# Signal that capture is done
done_event.set()

# Wait for the capture thread to finish
capture_thread.join()

print(f"Robot movement completed and joint states saved to {filename}.")
