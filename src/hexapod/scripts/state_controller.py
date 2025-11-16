#!/usr/bin/env python3
"""
State Machine Node - Manages gait cycle and calculates phase for each leg
INPUT: /hexapod/gait_parameters (Float64MultiArray - gait configuration)
INPUT: /hexapod/body_velocity (Twist - body velocity)
OUTPUT: /hexapod/leg_{1-6}/phase_info (Float64MultiArray - [phase_type, progress, leg_phase])
Frequency: 50 Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        # Parameters
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('num_legs', 6)
        
        control_rate = self.get_parameter('control_rate').value
        self.num_legs = self.get_parameter('num_legs').value
        self.dt = 1.0 / control_rate
        
        # State variables
        self.gait_type = 0
        self.step_height = 0.03
        self.step_length = 0.05
        self.cycle_time = 1.0
        self.duty_factor = 0.5
        self.body_velocity = Twist()
        
        # Phase tracking
        self.gait_phase = 0.0  # Global gait phase [0, 1]
        self.time_in_cycle = 0.0
        
        # Gait configurations with phase offsets
        self.gait_configs = {
            0: {'name': 'tripod', 'duty_factor': 0.5},
            1: {'name': 'wave', 'duty_factor': 0.83},
            2: {'name': 'ripple', 'duty_factor': 0.67}
        }
        
        # INPUT: Subscribers
        self.gait_params_sub = self.create_subscription(
            Float64MultiArray, '/hexapod/gait_parameters', self.gait_params_callback, 10)
        self.body_vel_sub = self.create_subscription(
            Twist, '/hexapod/body_velocity', self.body_velocity_callback, 10)
        
        # OUTPUT: Publishers - phase info (one per leg)
        self.phase_pubs = {}
        for i in range(1, self.num_legs + 1):
            self.phase_pubs[i] = self.create_publisher(
                Float64MultiArray, f'/hexapod/leg_{i}/phase_info', 10)
        
        # Timer - 50 Hz
        self.timer = self.create_timer(self.dt, self.update_state)
        
        self.get_logger().info(f'State Machine initialized with {self.num_legs} legs')
    
    def gait_params_callback(self, msg):
        """INPUT: Receive gait parameters [gait_type, step_height, step_length, cycle_time, duty_factor]"""
        if len(msg.data) >= 5:
            self.gait_type = int(msg.data[0])
            self.step_height = float(msg.data[1])
            self.step_length = float(msg.data[2])
            self.cycle_time = float(msg.data[3])
            self.duty_factor = float(msg.data[4])
            
            self.get_logger().debug(
                f'Updated gait params: type={self.gait_type}, '
                f'step_height={self.step_height:.3f}, step_length={self.step_length:.3f}, '
                f'cycle_time={self.cycle_time:.3f}, duty_factor={self.duty_factor:.3f}'
            )
    
    def body_velocity_callback(self, msg):
        """INPUT: Receive body velocity"""
        self.body_velocity = msg
    
    def _get_leg_phase_offset(self, leg_id, gait_type):
        """Calculate phase offset for each leg based on gait pattern"""
        # Leg arrangement (typical hexapod):
        # Front: L1(1), R1(2)
        # Middle: L2(3), R2(4)  
        # Rear: L3(5), R3(6)
        
        if gait_type == 0:  # Tripod gait
            # Group 1: L1, R2, L3 (phase 0)
            # Group 2: R1, L2, R3 (phase 0.5)
            if leg_id in [1, 4, 5]:  # L1, R2, L3
                return 0.0
            else:
                return 0.5
        
        elif gait_type == 1:  # Wave gait
            # Sequential: each leg offset by 1/6
            phase_offsets = [0.0, 0.5, 1/6, 4/6, 2/6, 5/6]
            return phase_offsets[leg_id - 1]
        
        elif gait_type == 2:  # Ripple gait
            # Three groups of two legs
            phase_offsets = [0.0, 2/3, 1/3, 0.0, 2/3, 1/3]
            return phase_offsets[leg_id - 1]
        
        return 0.0
    
    def _calculate_phase_type(self, leg_phase, duty_factor):
        """Determine if leg is in stance (0) or swing (1) phase"""
        # Phase 0 to duty_factor = stance
        # Phase duty_factor to 1.0 = swing
        if leg_phase < duty_factor:
            return 0  # Stance
        else:
            return 1  # Swing
    
    def _calculate_progress(self, leg_phase, duty_factor, phase_type):
        """Calculate progress within current phase [0, 1]"""
        if phase_type == 0:  # Stance
            # Progress from 0 to duty_factor
            progress = leg_phase / duty_factor if duty_factor > 0 else 0.0
        else:  # Swing
            # Progress from duty_factor to 1.0
            swing_duration = 1.0 - duty_factor
            progress = (leg_phase - duty_factor) / swing_duration if swing_duration > 0 else 0.0
        
        return np.clip(progress, 0.0, 1.0)
    
    def update_state(self):
        """OUTPUT: Update gait cycle and phases - 50 Hz"""
        # Update global gait phase
        if self.cycle_time > 0:
            self.gait_phase += self.dt / self.cycle_time
            self.time_in_cycle += self.dt
            
            # Wrap phase to [0, 1]
            if self.gait_phase >= 1.0:
                self.gait_phase -= 1.0
                self.time_in_cycle = 0.0
        
        # Calculate and publish phase info for each leg
        for leg_id in range(1, self.num_legs + 1):
            # Get leg-specific phase offset
            phase_offset = self._get_leg_phase_offset(leg_id, self.gait_type)
            
            # Calculate leg phase with offset
            leg_phase = (self.gait_phase + phase_offset) % 1.0
            
            # Determine phase type (stance or swing)
            phase_type = self._calculate_phase_type(leg_phase, self.duty_factor)
            
            # Calculate progress within phase
            progress = self._calculate_progress(leg_phase, self.duty_factor, phase_type)
            
            # Publish phase info: [phase_type, progress, leg_phase]
            phase_msg = Float64MultiArray()
            phase_msg.data = [
                float(phase_type),
                float(progress),
                float(leg_phase)
            ]
            self.phase_pubs[leg_id].publish(phase_msg)
        
        self.get_logger().debug(
            f'Global phase: {self.gait_phase:.3f}, time: {self.time_in_cycle:.3f}s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()