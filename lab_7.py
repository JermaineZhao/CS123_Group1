from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np

IMAGE_WIDTH = 1400

# TODO: Add your new constants here

TIMEOUT = 2.0 #TODO threshold in timer_callback
SEARCH_YAW_VEL = 0.5 #TODO searching constant
TRACK_FORWARD_VEL = 0.5 #TODO tracking constant
KP = 1.0 #TODO proportional gain for tracking

class State(Enum):
    SEARCH = 0
    TRACK = 1

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = State.TRACK

        # TODO: Add your new member variables here
        self.kp = KP # TODO
        self.closest_normalized_bbox_x = 0.0
        self.last_detect_time = None
        self.detection_in_progress = False 

    def detection_callback(self, msg):
        """
        Determine which of the HAILO detections is the most central detected object
        """
        # TODO: Part 1
        # breakpoint()
        most_centered_x = None
        # dist_to_center = 100
        # for detection in msg.detections:
        #     bbox_x = detection.bbox.center.position.x
        # # print(bbox_x)
        #     normalized_bbox_x = (bbox_x - IMAGE_WIDTH/2) / (IMAGE_WIDTH/2)
        #     # print(normalized_bbox_x)
        #     if abs(normalized_bbox_x) < dist_to_center:
        #         dist_to_center = abs(normalized_bbox_x)
        #         most_centered_x = normalized_bbox_x
        #         self.closest_mormalized_bbox_x = normalized_bbox_x
        # self.last_detect_time = self.timer.clock.now()
        # print(self.closest_normalized_bbox_x)

        dist_to_center = 10000
        self.detection_in_progress = True
        print("BB")
        for detection in msg.detections:
            bbox_x = detection.bbox.center.position.x
            normalized_bbox_x = (bbox_x - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2)
            print("NBX:", normalized_bbox_x)
            # Find the closest detection to the center
            if abs(normalized_bbox_x) < dist_to_center:
                print("aAAAAAAAAAA")
                dist_to_center = abs(normalized_bbox_x)
                self.closest_normalized_bbox_x = normalized_bbox_x
        
        self.last_detect_time = self.timer.clock.now()
        print(self.closest_normalized_bbox_x)

        # breakpoint()


    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
        """
        time_since_last_detection = (self.get_clock().now() - self.last_detect_time).seconds_nanoseconds()[0]
        if time_since_last_detection > TIMEOUT: # TODO: Part 3.2
            if self.detection_in_progress:
                self.state = State.TRACK
            else:
                self.state = State.SEARCH
        else:
            self.state = State.TRACK

        yaw_command = 0.0
        forward_vel_command = 0.0

        if self.state == State.SEARCH:
            yaw_command = SEARCH_YAW_VEL # TODO: Part 3.1
            forward_vel_command = 0.0
        elif self.state == State.TRACK:
            # TODO: Part 2 / 3.4
            yaw_command = -self.closest_normalized_bbox_x * self.kp
            forward_vel_command = TRACK_FORWARD_VEL
            print("yaw: ", yaw_command, "next: ", self.closest_normalized_bbox_x)

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        self.command_publisher.publish(cmd)

def main():
    rclpy.init()
    state_machine_node = StateMachineNode()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        zero_cmd = Twist()
        state_machine_node.command_publisher.publish(zero_cmd)

        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
