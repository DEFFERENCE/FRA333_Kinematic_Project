#!/usr/bin/env python3
"""
Set Point Generator Node - Generates discrete foot positions (ONE PER LEG)
INPUT: /hexapod/leg_X/phase_info (Float64MultiArray - [phase_type, progress, leg_phase])
INPUT: /hexapod/body_velocity (Twist - body velocity)
INPUT: /hexapod/gait_parameters (Float64MultiArray - [gait_type, step_height, step_length, cycle_time, duty_factor])
OUTPUT: /hexapod/leg_X/end_effector_setpoint (PointStamped - discrete foot position)
Frequency: 50 Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class SetPointGenerator(Node):
    def __init__(self):
        super().__init__('set_point_generator')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('stance_x', 0.15)
        self.declare_parameter('stance_y', 0.0)
        self.declare_parameter('stance_z', -0.05)
        self.declare_parameter('update_rate', 50.0)
        
        self.leg_id = self.get_parameter('leg_id').value
        self.stance_x = self.get_parameter('stance_x').value
        self.stance_y = self.get_parameter('stance_y').value
        self.stance_z = self.get_parameter('stance_z').value
        update_rate = self.get_parameter('update_rate').value
        
        # State variables
        self.phase_type = 0  # 0: stance, 1: swing
        self.progress = 0.0
        self.leg_phase = 0.0
        self.body_velocity = Twist()
        self.step_height = 0.03
        self.step_length = 0.05
        
        # Current position
        self.current_position = np.array([self.stance_x, self.stance_y, self.stance_z])
        
        # INPUT: Subscribers
        self.phase_sub = self.create_subscription(
            Float64MultiArray, f'/hexapod/leg_{self.leg_id}/phase_info', 
            self.phase_callback, 10)
        self.body_vel_sub = self.create_subscription(
            Twist, '/hexapod/body_velocity', self.body_velocity_callback, 10)
        self.gait_params_sub = self.create_subscription(
            Float64MultiArray, '/hexapod/gait_parameters', 
            self.gait_params_callback, 10)
        
        # OUTPUT: Publisher - setpoint for THIS leg only
        self.setpoint_pub = self.create_publisher(
            PointStamped, f'/hexapod/leg_{self.leg_id}/end_effector_setpoint', 10)
        
        # Timer - 50 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.generate_setpoint)
        
        self.get_logger().info(f'Set Point Generator for leg {self.leg_id} initialized')
    
    def phase_callback(self, msg):
        """INPUT: Receive phase information [phase_type, progress, leg_phase]"""
        if len(msg.data) >= 3:
            self.phase_type = int(msg.data[0])
            self.progress = float(msg.data[1])
            self.leg_phase = float(msg.data[2])
    
    def body_velocity_callback(self, msg):
        """INPUT: Receive body velocity"""
        self.body_velocity = msg
    
    def gait_params_callback(self, msg):
        """INPUT: Receive gait parameters [gait_type, step_height, step_length, cycle_time, duty_factor]"""
        if len(msg.data) >= 3:
            self.step_height = float(msg.data[1])
            self.step_length = float(msg.data[2])
    
    def _generate_stance_position(self, progress):
        """Generate position during stance phase (foot on ground, moving backward)"""
        # Stance: foot moves from front to back (opposite to body motion)
        # Progress 0 -> 1: move from +step_length/2 to -step_length/2
        
        x = self.stance_x + self.step_length / 2.0 - progress * self.step_length
        y = self.stance_y
        z = self.stance_z  # Stay on ground
        
        return np.array([x, y, z])
    
    def _generate_swing_position(self, progress):
        """Generate position during swing phase (foot in air, moving forward)"""
        # Swing: foot lifts, moves forward, and lands
        # Progress 0 -> 1: move from back to front with parabolic arc
        
        # X: move from -step_length/2 to +step_length/2
        x = self.stance_x - self.step_length / 2.0 + progress * self.step_length
        
        # Y: lateral movement (typically none for forward walking)
        y = self.stance_y
        
        # Z: parabolic arc (peaks at progress = 0.5)
        # Using formula: z = 4 * h * t * (1 - t) where h is step_height
        z = self.stance_z + self.step_height * 4 * progress * (1 - progress)
        
        return np.array([x, y, z])
    
    def generate_setpoint(self):
        """OUTPUT: Generate setpoint based on phase - 50 Hz"""
        # Generate position based on phase type
        if self.phase_type == 0:  # Stance
            position = self._generate_stance_position(self.progress)
        else:  # Swing
            position = self._generate_swing_position(self.progress)
        
        self.current_position = position
        
        # Publish setpoint
        setpoint_msg = PointStamped()
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = f'leg_{self.leg_id}_base'
        setpoint_msg.point.x = float(position[0])
        setpoint_msg.point.y = float(position[1])
        setpoint_msg.point.z = float(position[2])
        
        self.setpoint_pub.publish(setpoint_msg)
        
        self.get_logger().debug(
            f'Leg {self.leg_id}: phase={self.phase_type}, progress={self.progress:.3f}, '
            f'pos=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SetPointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()