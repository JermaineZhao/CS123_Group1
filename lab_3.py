import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.optimize import minimize

np.set_printoptions(precision=3, suppress=True)

Kp = 0.5
Kd = 0.01

class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )

        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 60   # 10 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None

        self.ee_triangle_positions = np.array([
            [0.05, 0.0, -0.12],  # Touchdown
            [-0.05, 0.0, -0.12], # Liftoff
            [0.0, 0.0, -0.06]    # Mid-swing
        ])

        center_to_rf_hip = np.array([0.07500, -0.08350, 0])
        self.ee_triangle_positions = self.ee_triangle_positions + center_to_rf_hip
        self.current_target = 0
        self.t = 0

    def listener_callback(self, msg):
        joints_of_interest = ['leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3']
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def forward_kinematics(self, theta1, theta2, theta3):
        ################################################################################################
        def rotation_x(angle):
            # rotation about the x-axis implemented for you
            return np.array([
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1]
            ])

        def rotation_y(angle):
            ## TODO: Implement the rotation matrix about the y-axis
            return np.array([
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0],
                [0, 0, 0, 1]
            ])
        
        def rotation_z(angle):
            ## TODO: Implement the rotation matrix about the z-axis
            return np.array([
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

        def translation(x, y, z):
            ## TODO: Implement the translation matrix
            return np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])
            
        # T_0_1 (base_link to leg_front_r_1)
        T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1)

        # T_1_2 (leg_front_r_1 to leg_front_r_2)
        ## TODO: Implement the transformation matrix from leg_front_r_1 to leg_front_r_2
        T_1_2 = translation(0, 0, 0.039) @ rotation_y(-np.pi/2) @ rotation_z(theta2)

        # T_2_3 (leg_front_r_2 to leg_front_r_3)
        ## TODO: Implement the transformation matrix from leg_front_r_2 to leg_front_r_3
        T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(np.pi/2) @ rotation_z(theta3)

        # T_3_ee (leg_front_r_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, 0.018)

        # TODO: Compute the final transformation. T_0_ee is a concatenation of the previous transformation matrices
        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        # TODO: Extract the end-effector position. The end effector position is a 3x3 matrix (not in homogenous coordinates)
        end_effector_position = T_0_ee[:3, -1]

        return end_effector_position
        ################################################################################################

    def inverse_kinematics(self, target_ee, initial_guess=[0,0,0]):
        def cost_function(theta):
            # Compute the cost function and the L1 norm of the error
            # return the cost and the L1 norm of the error
            ################################################################################################
            # TODO: Implement the cost function
            # HINT: You can use the * notation on a list to "unpack" a list
            end_effector_position = self.forward_kinematics(*theta)
            l1 = np.abs(end_effector_position - target_ee)
            cost = np.sum(np.square(l1))

            ################################################################################################
            return cost, l1

        def gradient(theta, epsilon=1e-3):
            grad = np.zeros_like(theta)
            for i in range(len(theta)):
                theta_eps_plus = np.copy(theta)
                theta_eps_minus = np.copy(theta)
                theta_eps_plus[i] += epsilon
                theta_eps_minus[i] -= epsilon
                cost_plus, _ = cost_function(theta_eps_plus)
                cost_minus, _ = cost_function(theta_eps_minus)
                grad[i] = (cost_plus - cost_minus) / (2*epsilon)
            return grad
            
            # Compute the gradient of the cost function using finite differences
            ################################################################################################
            # TODO: Implement the gradient computation
            ################################################################################################
        theta = np.array(initial_guess)
        learning_rate = 5.5# TODO: Set the learning rate, 5.5
        max_iterations = 20 # TODO: Set the maximum number of iterations
        tolerance = 0.001 # TODO: Set the tolerance for the L1 norm of the error

        cost_l = []
        for _ in range(max_iterations):
            grad = gradient(theta)
            # for j in range(len(theta)):
            theta -= grad * learning_rate
            cur_cost, l1 = cost_function(theta)
            cost_l.append(cur_cost)
            val = np.mean(l1)
            if val < tolerance:
                print("YESSS")
                break 
        print(theta)
            
        # print("touch down")
                    

            # Update the theta (parameters) using the gradient and the learning rate
            ################################################################################################
            # TODO: Implement the gradient update. Use the cost function you implemented, and use tolerance t
            # to determine if IK has converged
            # TODO (BONUS): Implement the (quasi-)Newton's method instead of finite differences for faster convergence
            ################################################################################################
        
        # # BONUS Method
        # def cost(theta):
        #     ee_pos = self.forward_kinematics(*theta)
        #     return np.sum((ee_pos - target_ee) ** 2)  # L2 norm squared

        # # Use the BFGS optimization method
        # result = minimize(
        #     cost,
        #     x0=np.array(initial_guess),
        #     method='BFGS',
        #     options={
        #         'gtol': 1e-4,
        #         'disp': False,
        #         'maxiter': 100
        #     }
        # )

        # if not result.success:
        #     self.get_logger().warn(f"IK did not converge: {result.message}")

        # return result.x  # optimized joint angles


        # print(f'Cost: {cost_l}') # Use to debug to see if you cost function converges within max_iterations

        return theta

    def interpolate_triangle(self, t):
        # Intepolate between the three triangle positions in the self.ee_triangle_positions
        # based on the current time t
        ################################################################################################
        # TODO: Implement the interpolation function

        A, B, C = self.ee_triangle_positions
        phase = t % 3
        if phase < 1.0:
            alpha = phase
            target = (1 - alpha) * A + alpha * B
        elif phase < 2.0:
            alpha = phase - 1.0
            target = (1 - alpha) * B + alpha * C
        else:
            alpha = phase - 2.0
            target = (1 - alpha) * C + alpha * A
        
        print(t)

        return target

        ################################################################################################

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            target_ee = self.interpolate_triangle(self.t)
            self.target_joint_positions = self.inverse_kinematics(target_ee, self.joint_positions)
            current_ee = self.forward_kinematics(*self.joint_positions)
            # print(f"Solving IK for phase: {self.t % 3:.2f}, Target EE: {target_ee}")


            # update the current time for the triangle interpolation
            ################################################################################################
            # TODO: Implement the time update
            self.t += self.ik_timer_period * 5
            ################################################################################################
            
            # self.get_logger().info(f'Target EE: {target_ee}, Current EE: {current_ee}, Target Angles: {self.target_joint_positions}, Target Angles to EE: {self.forward_kinematics(*self.target_joint_positions)}, Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:
            error = self.target_joint_positions - self.joint_positions
            d_error = -self.joint_velocities

            command = Kp * error + Kd * d_error

            command_msg = Float64MultiArray()
            command_msg.data = command.tolist()
            self.command_publisher.publish(command_msg)
        # if self.target_joint_positions is not None:

        #     command_msg = Float64MultiArray()
        #     command_msg.data = self.target_joint_positions.tolist()
        #     self.command_publisher.publish(command_msg)

def main():
    rclpy.init()
    inverse_kinematics = InverseKinematics()
    
    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0, 0.0, 0.0]
        inverse_kinematics.command_publisher.publish(zero_torques)
        
        inverse_kinematics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()