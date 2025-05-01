import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(precision=3, suppress=True)

def rotation_x(angle):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
    return np.array([
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1]
            ])

def rotation_y(angle):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
    return np.array([
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0],
                [0, 0, 0, 1]
            ])

def rotation_z(angle):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
    return np.array([
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

def translation(x, y, z):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
    return np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])

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

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None
        self.counter = 0

        # Trotting gate positions, already implemented
        # FAST
        
        touch_down_position = np.array([0.05, 0.0, -0.14])
        stand_position_1 = np.array([0.025, 0.0, -0.14])
        stand_position_2 = np.array([0.0, 0.0, -0.14])
        stand_position_3 = np.array([-0.025, 0.0, -0.14])
        liftoff_position = np.array([-0.05, 0.0, -0.14])
        mid_swing_position = np.array([0.0, 0.0, -0.05])
        

        # SLOW
        '''
        touch_down_position = np.array([0.005, 0.0, -0.14])
        stand_position_1 = np.array([0.0025, 0.0, -0.14])
        stand_position_2 = np.array([0.0, 0.0, -0.14])
        stand_position_3 = np.array([-0.0025, 0.0, -0.14])
        liftoff_position = np.array([-0.005, 0.0, -0.14])
        mid_swing_position = np.array([0.0, 0.0, -0.05])
        '''
        
        
        ## trotting
        # TODO: Implement each leg's trajectory in the trotting gait.
        rf_ee_offset = np.array([0.06, -0.09, 0])
        rf_ee_triangle_positions = np.array([
            ################################################################################################
            stand_position_2,
            stand_position_3,
            liftoff_position,
            mid_swing_position,
            touch_down_position,
            stand_position_1,
            ################################################################################################,
        ]) + rf_ee_offset
        
        lf_ee_offset = np.array([0.06, 0.09, 0])
        lf_ee_triangle_positions = np.array([
            ################################################################################################
            mid_swing_position,
            touch_down_position,
            stand_position_1,
            stand_position_2,
            stand_position_3,
            liftoff_position,
            ################################################################################################
        ]) + lf_ee_offset
        
        rb_ee_offset = np.array([-0.11, -0.09, 0])
        rb_ee_triangle_positions = np.array([
            ################################################################################################
            mid_swing_position,
            touch_down_position,
            stand_position_1,
            stand_position_2,
            stand_position_3,
            liftoff_position,
            ################################################################################################
        ]) + rb_ee_offset
        
        lb_ee_offset = np.array([-0.11, 0.09, 0])
        lb_ee_triangle_positions = np.array([
            ################################################################################################
            stand_position_2,
            stand_position_3,
            liftoff_position,
            mid_swing_position,
            touch_down_position,
            stand_position_1,
            ################################################################################################
        ]) + lb_ee_offset


        self.ee_triangle_positions = [rf_ee_triangle_positions, lf_ee_triangle_positions, rb_ee_triangle_positions, lb_ee_triangle_positions]
        self.fk_functions = [self.fr_leg_fk, self.fl_leg_fk, self.br_leg_fk, self.bl_leg_fk]

        self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()
        #print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
        #print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')


        self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 60   # 10 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)


    def fr_leg_fk(self, theta):
        # Already implemented in Lab 2
        T_RF_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def fl_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
<<<<<<< HEAD
        T_RF_0_1 = translation(0.07500, 0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
=======
        T_RF_0_1 = translation(0.07500, 0.08350, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, -0.01800)
>>>>>>> 5bb41cd (Code for Lab 4)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]
    
    
    def br_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
        T_RF_0_1 = translation(-0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def bl_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
<<<<<<< HEAD
        T_RF_0_1 = translation(-0.07500, 0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
=======
        T_RF_0_1 = translation(-0.07500, 0.08350, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(-theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, -0.01800)
>>>>>>> 5bb41cd (Code for Lab 4)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def forward_kinematics(self, theta):
        return np.concatenate([self.fk_functions[i](theta[3*i: 3*i+3]) for i in range(4)])

    def listener_callback(self, msg):
        joints_of_interest = [
            'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3', 
            'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3', 
            'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3', 
            'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3'
        ]
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def inverse_kinematics_single_leg(self, target_ee, leg_index, initial_guess=[0, 0, 0]):
        leg_forward_kinematics = self.fk_functions[leg_index]

        def cost_function(theta):
            # current_position = leg_forward_kinematics(theta)
            ################################################################################################
            # TODO: [already done] paste lab 3 inverse kinematics here
<<<<<<< HEAD
            end_effector_position = self.forward_kinematics(*theta)
            l1 = np.abs(end_effector_position - target_ee)
=======
            #end_effector_position = self.forward_kinematics(*theta)
            l1 = np.abs(current_position - target_ee)
>>>>>>> 5bb41cd (Code for Lab 4)
            cost = np.sum(np.square(l1))
            return cost, l1
            
            

        def gradient(theta, epsilon=1e-3):
            # grad = np.zeros(3)
            ################################################################################################
            # TODO: [already done] paste lab 3 inverse kinematics here
            ################################################################################################
<<<<<<< HEAD
            grad = np.zeros_like(theta)
=======
            # grad = np.zeros_like(theta)
>>>>>>> 5bb41cd (Code for Lab 4)
            for i in range(len(theta)):
                theta_eps_plus = np.copy(theta)
                theta_eps_minus = np.copy(theta)
                theta_eps_plus[i] += epsilon
                theta_eps_minus[i] -= epsilon
                cost_plus, _ = cost_function(theta_eps_plus)
                cost_minus, _ = cost_function(theta_eps_minus)
                grad[i] = (cost_plus - cost_minus) / (2*epsilon)
            return grad

<<<<<<< HEAD
        theta = np.array(initial_guess)
        learning_rate = 5.5 # TODO: Set the learning rate, 5.5
        max_iterations = 20 # TODO: Set the maximum number of iterations
=======
        # initial_guess=[0.1, 0.1, 0.1]
        theta = np.array(initial_guess).astype(np.float64)
        #print("theta is:", theta)
        learning_rate = 10 # TODO:  the learning rate, 5.5
        max_iterations = 50 # TODO: Set the maximum number of iterations
>>>>>>> 5bb41cd (Code for Lab 4)
        tolerance = 0.001 # TODO: Set the tolerance for the L1 norm of the error

        cost_l = []
        flag = False
        while(not flag):
        # for _ in range(max_iterations):
            ################################################################################################
            # TODO: [already done] paste lab 3 inverse kinematics here
            ################################################################################################
            grad = gradient(theta)
            # for j in range(len(theta)):
<<<<<<< HEAD
=======
            # print(theta)
            # print(grad)
            # print(type(grad))
            # print(grad)
>>>>>>> 5bb41cd (Code for Lab 4)
            theta -= grad * learning_rate
            cur_cost, l1 = cost_function(theta)
            cost_l.append(cur_cost)
            val = np.mean(l1)
            if val < tolerance:
<<<<<<< HEAD
                print("YESSS")
                break 
        print(theta)
=======
                flag = True
                #print("YESSS")
                break 
        # print(theta)
>>>>>>> 5bb41cd (Code for Lab 4)

        return theta



    def interpolate_triangle(self, t, leg_index):
        ################################################################################################
        # TODO: implement interpolation for all 4 legs here
        ################################################################################################
        ee_triangle_position = self.ee_triangle_positions[leg_index]

<<<<<<< HEAD
        
        A, B, C, D, E, F = ee_triangle_position
        phase = t % 6
        if phase < 1.0:
            alpha = phase
            target = (1 - alpha) * A + alpha * B
        elif phase < 2.0:
            alpha = phase - 1.0
            target = (1 - alpha) * B + alpha * C
        elif phase < 3.0:
            alpha = phase - 2.0
            target = (1 - alpha) * C + alpha * D
        elif phase < 4.0:
            alpha = phase - 3.0
            target = (1 - alpha) * D + alpha * E
        elif phase < 5.0:
            alpha = phase - 4.0
            target = (1 - alpha) * E + alpha * F
        else:
            alpha = phase - 5.0
            target = (1 - alpha) * F + alpha * A
        
        # print(t)

        return target
=======
        '''
        A, B, C, D, E, F = ee_triangle_position
        phase = t % 1.
        if phase < 1/6:
            alpha = phase
            target = (1 - alpha) * A + alpha * B
        elif phase < 1/3:
            alpha = phase - 1/6
            target = (1 - alpha) * B + alpha * C
        elif phase < 1/2:
            alpha = phase - 2/6
            target = (1 - alpha) * C + alpha * D
        elif phase < 2/3:
            alpha = phase - 3/6
            target = (1 - alpha) * D + alpha * E
        elif phase < 5/6:
            alpha = phase - 4/6
            target = (1 - alpha) * E + alpha * F
        else:
            alpha = phase - 5/6
            target = (1 - alpha) * F + alpha * A
        
        # print(t)
        return target
        '''
        
        t = t*12%12.
        t_idx = int(t)
        t_res = t - t_idx
        return ee_triangle_position[t_idx % 6] + t_res * (ee_triangle_position[(t_idx + 1) % 6] - ee_triangle_position[t_idx % 6])
>>>>>>> 5bb41cd (Code for Lab 4)

    def cache_target_joint_positions(self):
        # Calculate and store the target joint positions for a cycle and all 4 legs
        target_joint_positions_cache = []
        target_ee_cache = []
        for leg_index in range(4):
            print(f"Leg: {leg_index}")
            target_joint_positions_cache.append([])
            target_ee_cache.append([])
            target_joint_positions = [0] * 3
            for t in np.arange(0, 1, 0.02):
                print(t)
                target_ee = self.interpolate_triangle(t, leg_index)
                target_joint_positions = self.inverse_kinematics_single_leg(target_ee, leg_index, initial_guess=target_joint_positions)

                target_joint_positions_cache[leg_index].append(target_joint_positions)
                target_ee_cache[leg_index].append(target_ee)

        # (4, 50, 3) -> (50, 12)
        target_joint_positions_cache = np.concatenate(target_joint_positions_cache, axis=1)
        target_ee_cache = np.concatenate(target_ee_cache, axis=1)
        
        return target_joint_positions_cache, target_ee_cache

    def get_target_joint_positions(self):
        target_joint_positions = self.target_joint_positions_cache[self.counter]
        target_ee = self.target_ee_cache[self.counter]
        self.counter += 1
        if self.counter >= self.target_joint_positions_cache.shape[0]:
            self.counter = 0
        return target_ee, target_joint_positions

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            target_ee, self.target_joint_positions = self.get_target_joint_positions()
            current_ee = self.forward_kinematics(self.joint_positions)

            self.get_logger().info(
                f'Target EE: {target_ee}, \
                Current EE: {current_ee}, \
                Target Angles: {self.target_joint_positions}, \
                Target Angles to EE: {self.forward_kinematics(self.target_joint_positions)}, \
                Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:
            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)

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
        zero_torques.data = [0.0] * 12
        inverse_kinematics.command_publisher.publish(zero_torques)
        
        inverse_kinematics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
