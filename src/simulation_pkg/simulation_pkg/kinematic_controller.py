#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        
        # Publisher for robot velocity commands (global frame)
        self.cmd_vel_pub = self.create_publisher(Twist, '/o1/cmd_vel', 10)
        # Publisher for local cmd_vel
        self.cmd_vel_local_pub = self.create_publisher(Twist, 'cmd_vel_local', 10)
        
        # Subscribers for current state and waypoints
        self.create_subscription(Float32MultiArray, 'o1_data', self.current_state_callback, 10)
        self.create_subscription(Float32MultiArray, '/o1/target_pos', self.target_state_callback, 10)
        
        # Current robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        # Previous waypoint (point A)
        self.prev_waypoint_x = 0.0
        self.prev_waypoint_y = 0.0
        
        # Current waypoint (point B)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.target_vx = 0.1
        self.target_vy = 0.0
        self.target_omega = 0.0
        self.idx = 0
        self.path_length = 0
        
        # Controller parameters (tunable)
        self.max_linear_accel = 1    # m/s²
        self.max_linear_velocity = 1 # m/s
        self.max_angular_accel = 0.5   # rad/s²
        self.position_tolerance = 0.01 # m
        self.final_position_tolerance = 0.005  # tighter tolerance for final damping
        self.orientation_tolerance = 0.05 # rad
        self.min_velocity = 0.05       # m/s (avoid division by near-zero)
        self.min_time = 0.1            # s (minimum time estimate)
        self.epsilon = 1e-6            # small number for comparisons
        self.min_parallel_velocity = 0.2  # Minimum velocity to maintain in parallel direction
        self.damping_factor = 0.8      # Velocity damping factor for final position approach
        
        # Time tracking for dt calculation
        self.last_time = time.time()
        
        # Store last commanded velocities
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_omega = 0.0
        
        # For debugging
        self.debug_counter = 0
        
        # For trajectory calculations
        self.unit_parallel = (1.0, 0.0)  # Default direction
        self.unit_perp = (0.0, 1.0)      # Default perpendicular
        
        # Flag to track final approach
        self.final_approach = False
        
        # Set logging level
        self.debug_mode = True
        
        # PD control parameters for orientation
        self.Kp_theta = 2.0    # Proportional gain
        self.Kd_theta = 0.2    # Derivative gain (tunable)
        self.last_delta_theta = 0.0  # To store previous orientation error
        
        # PD gains for parallel direction
        self.Kp_parallel_pos = 3.0  # Proportional gain for position error (1/s²)
        self.Kd_parallel_vel = 6 # Derivative gain for velocity error (1/s)
        
        # PD gains for perpendicular direction
        self.Kp_perp_pos = 4  # Proportional gain for position error (1/s²)
        self.Kd_perp_vel = 10.0  # Derivative gain for velocity error (1/s)
        
        self.get_logger().info('Trajectory Controller initialized')

    def target_state_callback(self, msg):
        """Process new waypoint information"""
        if len(msg.data) < 6:
            self.get_logger().error('Received target message with insufficient data')
            return
            
        # Extract target state (point B)
        self.target_x = msg.data[0]
        self.target_y = msg.data[1]
        self.target_theta = msg.data[2]
        self.target_vx = msg.data[3]
        self.target_vy = msg.data[4]
        self.target_omega = msg.data[5]
        self.idx = msg.data[7]
        self.path_length = msg.data[8]

        # Extract previous waypoint (point A) if available
        if len(msg.data) > 10:
            self.prev_waypoint_x = msg.data[9]
            self.prev_waypoint_y = msg.data[10]
        else:
            self.prev_waypoint_x = self.current_x
            self.prev_waypoint_y = self.current_y
            
        # Reset final approach flag for new waypoint
        self.final_approach = False
            
        self.get_logger().info(f'New target: idx={self.idx}/{self.path_length}, '
                              f'pos=({self.target_x:.2f}, {self.target_y:.2f}), '
                              f'theta={self.target_theta:.2f}')

    def current_state_callback(self, msg):
        """Process updated robot state and compute control"""
        if len(msg.data) < 3:
            self.get_logger().error('Received state message with insufficient data')
            return
            
        # Calculate dt since last update
        current_time = time.time()
        dt = (current_time - self.last_time)  
        self.last_time = current_time
        
        # Ensure dt is reasonable
        if dt <= 0.0 or dt > 1.0:
            dt = 0.02  # Fallback to 50 Hz nominal
            
        # Update current state
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]
        self.current_theta = msg.data[2]
        
        # Update velocities if available
        if len(msg.data) > 8:
            self.current_vx = msg.data[7]
            self.current_vy = msg.data[8]
        
        # Compute and publish control command
        cmd = self.compute_trajectory_control(dt)
        self.cmd_vel_pub.publish(cmd)
        
        # Compute and publish local cmd_vel
        vx_global = cmd.linear.x
        vy_global = cmd.linear.y
        omega = cmd.angular.z
        theta = self.current_theta
        vx_local = vx_global * math.cos(theta) + vy_global * math.sin(theta)
        vy_local = -vx_global * math.sin(theta) + vy_global * math.cos(theta)
        local_cmd = Twist()
        local_cmd.linear.x = vx_local
        local_cmd.linear.y = vy_local
        local_cmd.angular.z = omega
        self.cmd_vel_local_pub.publish(local_cmd)
        
        # Store last commanded velocities
        self.last_cmd_vx = cmd.linear.x
        self.last_cmd_vy = cmd.linear.y
        self.last_cmd_omega = cmd.angular.z
        
        # Periodic debug output
        self.debug_counter += 1
        if self.debug_counter % 5 == 0:
            self.get_logger().info(
                f'State: pos=({self.current_x:.2f}, {self.current_y:.2f}), '
                f'vel=({self.current_vx:.2f}, {self.current_vy:.2f}), '
                f'speed = {math.sqrt(self.current_vx**2 + self.current_vy**2)}, '
                f'cmd=({self.last_cmd_vx:.2f}, {self.last_cmd_vy:.2f}), '
                f'theta={self.current_theta:.2f}'
            )

    def compute_trajectory_control(self, dt):
        """Compute control commands using PD control for accelerations"""
        twist = Twist()
        
        # Calculate distance to target
        dx_to_target = self.target_x - self.current_x
        dy_to_target = self.target_y - self.current_y
        distance_to_target = math.sqrt(dx_to_target**2 + dy_to_target**2)
        
        # Calculate current velocity magnitude
        current_velocity_magnitude = math.sqrt(self.last_cmd_vx**2 + self.last_cmd_vy**2)
        
        # Check if we've reached the final position
        if distance_to_target < self.position_tolerance:
            if current_velocity_magnitude < self.min_velocity:
                # Stop completely
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                
                # PD control for orientation
                delta_theta = self.normalize_angle(self.target_theta - self.current_theta)
                error_derivative = (delta_theta - self.last_delta_theta) / dt
                twist.angular.z = self.Kp_theta * delta_theta + self.Kd_theta * error_derivative
                twist.angular.z = self.clamp(twist.angular.z, -1.0, 1.0)
                self.last_delta_theta = delta_theta
                
                if abs(delta_theta) < self.orientation_tolerance:
                    self.get_logger().info('Target position and orientation reached')
            else:
                # Apply damping to stop smoothly
                damping = 0.5
                twist.linear.x = self.last_cmd_vx * damping
                twist.linear.y = self.last_cmd_vy * damping
                
                # PD control for orientation
                delta_theta = self.normalize_angle(self.target_theta - self.current_theta)
                error_derivative = (delta_theta - self.last_delta_theta) / dt
                twist.angular.z = self.Kp_theta * delta_theta + self.Kd_theta * error_derivative
                twist.angular.z = self.clamp(twist.angular.z, -1.0, 1.0)
                self.last_delta_theta = delta_theta
            
            return twist
        
        # Initiate final approach damping
        if distance_to_target < 5 * self.position_tolerance and not self.final_approach:
            self.final_approach = True
            self.get_logger().info('Final approach initiated, applying velocity damping')
        
        # Compute trajectory vectors
        self.compute_trajectory_vectors()
        
        # Project position and velocity onto trajectory
        parallel_dist, perp_dist, remaining_parallel_dist, traj_length = self.project_position_onto_trajectory()
        v_parallel, v_perp, target_v_parallel = self.project_velocity_onto_trajectory()
        
        # Calculate control accelerations using PD control
        a_parallel = self.calculate_parallel_acceleration(v_parallel, target_v_parallel, remaining_parallel_dist)
        a_perp = self.calculate_perpendicular_acceleration(perp_dist, v_perp)
        
        # Calculate new velocities
        new_vx, new_vy = self.calculate_velocity_commands(a_parallel, a_perp, dt)
        
        # Apply damping during final approach
        if self.final_approach:
            damping_factor = self.damping_factor * (1 - math.exp(-distance_to_target / self.position_tolerance))
            new_vx *= damping_factor
            new_vy *= damping_factor
            if self.debug_mode:
                self.get_logger().debug(f'Applying final approach damping: factor={damping_factor:.3f}')
        
        # Set velocity commands
        twist.linear.x = new_vx
        twist.linear.y = new_vy
        
        # PD control for orientation
        delta_theta = self.normalize_angle(self.target_theta - self.current_theta)
        error_derivative = (delta_theta - self.last_delta_theta) / dt
        twist.angular.z = self.Kp_theta * delta_theta + self.Kd_theta * error_derivative
        twist.angular.z = self.clamp(twist.angular.z, -1.0, 1.0)
        self.last_delta_theta = delta_theta
        
        return twist

    def compute_trajectory_vectors(self):
        """Compute unit vectors along and perpendicular to the trajectory"""
        dx_traj = self.target_x - self.prev_waypoint_x
        dy_traj = self.target_y - self.prev_waypoint_y
        traj_length = math.sqrt(dx_traj**2 + dy_traj**2)
        
        if traj_length < self.epsilon:
            dx_to_target = self.target_x - self.current_x
            dy_to_target = self.target_y - self.current_y
            distance_to_target = math.sqrt(dx_to_target**2 + dy_to_target**2)
            
            if distance_to_target > self.epsilon:
                self.unit_parallel = (dx_to_target / distance_to_target, 
                                     dy_to_target / distance_to_target)
            else:
                self.unit_parallel = (1.0, 0.0)
        else:
            self.unit_parallel = (dx_traj / traj_length, dy_traj / traj_length)
        
        self.unit_perp = (-self.unit_parallel[1], self.unit_parallel[0])
        
    def project_position_onto_trajectory(self):
        """Project current position onto trajectory line and calculate distances"""
        dx_from_prev = self.current_x - self.prev_waypoint_x
        dy_from_prev = self.current_y - self.prev_waypoint_y
        
        parallel_dist = dx_from_prev * self.unit_parallel[0] + dy_from_prev * self.unit_parallel[1]
        perp_dist = dx_from_prev * self.unit_perp[0] + dy_from_prev * self.unit_perp[1]
        
        dx_traj = self.target_x - self.prev_waypoint_x
        dy_traj = self.target_y - self.prev_waypoint_y
        traj_length = math.sqrt(dx_traj**2 + dy_traj**2)
        
        remaining_parallel_dist = traj_length - parallel_dist
        
        return parallel_dist, perp_dist, remaining_parallel_dist, traj_length
        
    def project_velocity_onto_trajectory(self):
        """Project velocities onto parallel and perpendicular directions"""
        v_parallel = self.last_cmd_vx * self.unit_parallel[0] + self.last_cmd_vy * self.unit_parallel[1]
        v_perp = self.last_cmd_vx * self.unit_perp[0] + self.last_cmd_vy * self.unit_perp[1]
        
        target_v_parallel = self.target_vx * self.unit_parallel[0] + self.target_vy * self.unit_parallel[1]
        
        if (self.path_length - self.idx) == 4:
            target_v_parallel *= 0.7
        elif (self.path_length - self.idx) == 3:
            target_v_parallel *= 0.5
        elif (self.path_length - self.idx) == 2:
            target_v_parallel *= 0.3
        elif (self.path_length - self.idx) == 1:
            target_v_parallel *= 0.1

        return v_parallel, v_perp, target_v_parallel
        
    def calculate_parallel_acceleration(self, v_parallel, target_v_parallel, remaining_parallel_dist):
        """Calculate acceleration along the trajectory direction using PD control"""
        e_pos = remaining_parallel_dist  # Position error
        e_vel = target_v_parallel - v_parallel  # Velocity error
        a_parallel = self.Kp_parallel_pos * e_pos + self.Kd_parallel_vel * e_vel
        a_parallel = self.clamp(a_parallel, -self.max_linear_accel, self.max_linear_accel)
        if self.debug_mode:
            self.get_logger().debug(
                f'PARALLEL: dist={remaining_parallel_dist:.3f}, v={v_parallel:.3f}, '
                f'target_v={target_v_parallel:.3f}, a={a_parallel:.3f}'
            )
        print(f'[PARALLEL] remaining_dist={remaining_parallel_dist:.3f}m, v_current={v_parallel:.3f}m/s, '
          f'v_target={target_v_parallel:.3f}m/s, accel={a_parallel:.3f}m/s²')
        return a_parallel
    
    def calculate_perpendicular_acceleration(self, perp_dist, v_perp):
        """Calculate acceleration perpendicular to the trajectory using PD control"""
        e_pos = perp_dist  # Position error (desired is 0)
        e_vel = -v_perp    # Velocity error (desired is 0, so error is -v_perp)
        a_perp = -self.Kp_perp_pos * e_pos - self.Kd_perp_vel * v_perp
        a_perp = self.clamp(a_perp, -self.max_linear_accel, self.max_linear_accel)
        if self.debug_mode:
            self.get_logger().debug(
                f'PERP: dist={perp_dist:.3f}, v={v_perp:.3f}, a={a_perp:.3f}'
            )
        print(f'[PERPENDICULAR] perp_error={perp_dist:.3f}m, v_perp={v_perp:.3f}m/s, '
          f'accel={a_perp:.3f}m/s²')
        return a_perp
        
    def calculate_velocity_commands(self, a_parallel, a_perp, dt):
        """Calculate new velocity commands based on accelerations"""
        accel_x = a_parallel * self.unit_parallel[0] + a_perp * self.unit_perp[0]
        accel_y = a_parallel * self.unit_parallel[1] + a_perp * self.unit_perp[1]

        new_vx = self.last_cmd_vx + accel_x * dt
        new_vy = self.last_cmd_vy + accel_y * dt
        
        vel_magnitude = math.sqrt(new_vx**2 + new_vy**2)
        
        if vel_magnitude > self.max_linear_velocity:
            scale_factor = self.max_linear_velocity / vel_magnitude
            new_vx *= scale_factor
            new_vy *= scale_factor
        
        return new_vx, new_vy
        
    def clamp(self, value, min_val, max_val):
        """Clamp a value between min and max"""
        return max(min(value, max_val), min_val)
        
    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    controller = TrajectoryController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()