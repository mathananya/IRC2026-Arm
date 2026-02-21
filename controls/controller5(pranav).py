#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import math

class AckermannDriveController(Node):
    def __init__(self):
        super().__init__('ackermann_drive_controller')

        # Subscribers
        self.sub_keyboard = self.create_subscription(
            String, '/keyBoard', self.keyboard_callback, 10)

        # Publishers
        self.pub_angles = self.create_publisher(Float32MultiArray, 'target_angles', 10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)

        # Rover Dimensions
        self.L = 1.0  # Wheelbase
        self.W = 0.6  # Track Width

        # States
        self.vx = 0.0
        self.center_steer = 0.0

        # Parameters
        self.lin_step = 0.1
        self.max_v = 1.0
        self.steer_step = 1.0

        # Continuous Publishing Timer (20Hz)
        self.timer = self.create_timer(0.05, self.publish_state)

        self.get_logger().info("Drive Controller Started. Streaming at 20Hz. Waiting for /keyBoard input...")

    def keyboard_callback(self, msg):
        key = msg.data

        # Drive Controls
        if key == 'w':
            self.vx = min(self.max_v, self.vx + self.lin_step)
        elif key == 's':
            self.vx = max(-self.max_v, self.vx - self.lin_step)

        # Steering Controls
        elif key == 'a':
            self.center_steer = max(self.center_steer - self.steer_step, -45.0)
        elif key == 'd':
            self.center_steer = min(self.center_steer + self.steer_step, 45.0)

        # Emergency Stop
        elif key == ' ':
            self.vx = 0.0
            self.center_steer = 0.0
            
        elif key == 'q':
            self.vx = 0.0
            self.center_steer = 0.0
            # Force one last stop publish before quitting
            self.publish_state() 
            rclpy.shutdown()
            return

    def publish_state(self):
        # ---------------------------------------------
        # 1. ACKERMANN PIVOT MATH
        # ---------------------------------------------
        alpha = math.radians(self.center_steer)

        if abs(alpha) < 0.001:  # Going straight
            theta_left = 0.0
            theta_right = 0.0
        else:
            R = self.L / math.tan(alpha)
            # Rear steering math
            theta_left = math.degrees(math.atan(self.L / (R + self.W / 2.0)))
            theta_right = math.degrees(math.atan(self.L / (R - self.W / 2.0)))

        # Wheels 0 and 2 fixed, Wheels 1 and 3 steering
        angles = [0.0, theta_left, 0.0, theta_right]

        msg_angles = Float32MultiArray()
        processed_angles = [float(a) for a in angles]
        # Hardware Inversion for Wheel 3
        processed_angles[3] = 360.0 - processed_angles[3]
        msg_angles.data = processed_angles
        self.pub_angles.publish(msg_angles)

        # ---------------------------------------------
        # 2. DRIVE VELOCITY MATH (DYNAMIC KINEMATICS FIX)
        # ---------------------------------------------
        msg_wheels = Float32MultiArray()

        if abs(self.center_steer) < 0.001:
            # Going straight
            v_lf = -self.vx
            v_lr = -self.vx
            v_rf = self.vx
            v_rr = self.vx
        else:
            alpha = math.radians(abs(self.center_steer))
            
            # 1. Calculate distance to ICC
            R = self.L / math.tan(alpha)
            
            # 2. Calculate turning radius for each wheel position
            r_fixed_inner = R - (self.W / 2.0)
            r_fixed_outer = R + (self.W / 2.0)
            r_steered_inner = math.sqrt(r_fixed_inner**2 + self.L**2)
            r_steered_outer = math.sqrt(r_fixed_outer**2 + self.L**2)
            
            # 3. Calculate Angular Velocity (omega)
            v_base = abs(self.vx)
            omega = v_base / r_steered_outer
            
            # 4. Calculate true continuous speeds (Replaces broken sine formulas)
            v_of = v_base                  # Steered Outer (Fastest)
            v_if = omega * r_steered_inner # Steered Inner
            v_ob = omega * r_fixed_outer   # Fixed Outer
            v_ib = omega * r_fixed_inner   # Fixed Inner
            
            if self.center_steer > 0:
                # TURNING LEFT (Retaining your debugged physical mapping)
                mag_lf = v_ob
                mag_rf = v_if
                mag_lr = v_of
                mag_rr = v_ib
            else:
                # TURNING RIGHT (Retaining your debugged physical mapping)
                mag_lf = v_if
                mag_rf = v_ob
                mag_lr = v_ib
                mag_rr = v_of

            # Apply directional signs (Left negative, Right positive)
            direction = 1.0 if self.vx >= 0 else -1.0
            
            v_lf = -mag_lf * direction
            v_lr = -mag_lr * direction
            v_rf = mag_rf * direction
            v_rr = mag_rr * direction

        msg_wheels.data = [float(v_lf), float(v_lr), float(v_rf), float(v_rr)]
        self.pub_wheels.publish(msg_wheels)
        
        # Dashboard Feedback
        print(f"\rSpeed: {self.vx:5.1f} | Input: {self.center_steer:5.1f}° | W1(L): {theta_left:5.1f}° | W3(R): {theta_right:5.1f}°    ", end="", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
