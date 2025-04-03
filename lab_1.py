import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
from collections import deque
import math

JOINT_NAME = 'leg_front_r_1'
####
####
KP = 1.0 # YOUR KP VALUE
KD = 0.1 # YOUR KD VALUE
####
####
LOOP_RATE = 200  # Hz
MAX_TORQUE = 3.0

class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('joint_state_subscriber')
        # Create a subscriber to the / joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.get_joint_info,
            10  # QoS profile history depth
        )
        self.subscription  # prevent unused variable warning

        # Publisher to the /forward_command_controller/commands topic
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )
        self.print_counter = 0
        self.pd_torque = 0
        self.joint_pos = 0
        self.joint_vel = 0
        self.target_joint_pos = 0
        self.target_joint_vel = 0
        # self.torque_history = deque(maxlen=DELAY)

        delay_seconds = 0.1
        control_freq = 200
        # In your initialization:
        self.delay_buffer_size = int(delay_seconds * control_freq)
        self.angle_buffer = deque(maxlen=self.delay_buffer_size)
        self.velocity_buffer = deque(maxlen=self.delay_buffer_size)

        

        # Create a timer to run pd_loop at the specified frequency
        self.create_timer(1.0 / LOOP_RATE, self.pd_loop)

    def get_target_joint_info(self):
        # YOUR CODE HERE

        # Bang-bang: oscillate back and forth between -0.5 and 0.5 (rads) every 3 seconds
        # --- bang-bang starts here ---
        # t = self.get_clock().now().nanoseconds / 1e9  # Current time stamp in seconds
        # period = 30

        # if int(t // period) % 2 == 0:
        #     target_joint_pos = 5
        # else:
        #     target_joint_pos = -5

        # target_joint_vel = 0    # We don't need vel for bang-bang
        # --- bang-bang ends here ---

        # PD control (sinusoidal)
        # --- PD starts here ---
        t = self.get_clock().now().nanoseconds / 1e9  # Current time stamp in seconds
        
        amplitude = math.pi / 4     # Rad
        frequency = 0.2
        omega = 2 * math.pi * frequency

        target_joint_pos = amplitude * math.sin(omega * t)
        target_joint_vel = amplitude * omega * math.cos(omega * t)

        # --- PD ends here ---

        return target_joint_pos, target_joint_vel

    def calculate_pd_torque(self, joint_pos, joint_vel, target_joint_pos, target_joint_vel):
        # YOUR CODE HERE

        # --- bang-bang starts here ---
        # position_err = target_joint_pos - joint_pos
        # threshold = 0.01    # rads

        # if position_err > threshold:
        #     torque = MAX_TORQUE
        # elif position_err < -threshold:
        #     torque = -MAX_TORQUE
        # else:
        #     torque = 0.0
        # --- bang-bang ends here ---

        # Add delay in your control loop:
        # self.angle_buffer.append(joint_pos)
        # self.velocity_buffer.append(joint_vel)
        # joint_pos = self.angle_buffer[0]
        # joint_vel = self.velocity_buffer[0]

        torque = KP * (target_joint_pos - joint_pos) + KD * (target_joint_vel - joint_vel)
        return torque

    def print_info(self):
        if self.print_counter == 0:
            print(f"Pos: {self.joint_pos:.2f}, Target Pos: {self.target_joint_pos:.2f}, Vel: {self.joint_vel:.2f}, Target Vel: {self.target_joint_vel:.2f}, Tor: {self.pd_torque:.2f}")
        self.print_counter += 1
        self.print_counter %= 50

    def get_joint_info(self, msg):
        joint_index = msg.name.index(JOINT_NAME)
        joint_pos = msg.position[joint_index]
        joint_vel = msg.velocity[joint_index]

        self.joint_pos = joint_pos
        self.joint_vel = joint_vel
    
        return joint_pos, joint_vel

    def pd_loop(self):
        self.target_joint_pos, self.target_joint_vel = self.get_target_joint_info()
        self.pd_torque = self.calculate_pd_torque(self.joint_pos, self.joint_vel, self.target_joint_pos, self.target_joint_vel)
        self.print_info()
        self.publish_torque(self.pd_torque)

    def publish_torque(self, torque=0.0):
        # Create a Float64MultiArray message with zero kp and kd values
        command_msg = Float64MultiArray()
        torque = np.clip(torque, -MAX_TORQUE, MAX_TORQUE)
        command_msg.data = [torque, 0.0, 0.0]  # Zero kp and kd values

        # Publish the message
        self.command_publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    joint_state_subscriber = JointStateSubscriber()

    # Keep the node running until interrupted
    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        print("Ctrl-C detected")
        joint_state_subscriber.publish_torque(0.0)
#    finally:
#        joint_state_subscriber.publish_torque(0.0)

    # Clean up and shutdown
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
