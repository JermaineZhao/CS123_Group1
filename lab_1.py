import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_NAME = 'leg_front_r_3' ### CHANGE THIS (OPTIONAL)
KP = 0 ### CHANGE THIS
KD = 0 ### CHANGE THIS

class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('joint_state_subscriber')
        # Create a subscriber to the /joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
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

    def calculate_pd_torque(self, joint_pos, joint_vel, joint_pos_desired=0.0, joint_vel_desired=0.0):
        ### CHANGE THIS
        return

    def listener_callback(self, msg):
        joint_index = msg.name.index(JOINT_NAME)
        joint_pos = msg.position[joint_index]
        joint_vel = msg.velocity[joint_index]
        if self.print_counter == 0:
            print("Pos: ", joint_pos, "Vel: ", joint_vel)
        self.print_counter += 1
        self.print_counter %= 10
        pd_torque = self.calculate_pd_torque(joint_pos, joint_vel, 1.0, 0.0)
        self.publish_torque(pd_torque)

    def publish_torque(self, torque=0.0):
        # Create a Float64MultiArray message with zero kp and kd values
        command_msg = Float64MultiArray()
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

    # Clean up and shutdown
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
