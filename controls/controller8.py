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
        self.pub_angles = self.create_publisher(
            Float32MultiArray, 'target_angles', 10)
        self.pub_wheels = self.create_publisher(
            Float32MultiArray, '/wheel_speeds', 10)

        # Rover Dimensions
        self.L = 1.0  # Wheelbase
        self.W = 0.6  # Track Width

        # States
        self.mode = 'ackermann'  # Options: 'ackermann', 'crab', 'pivot'
        self.vx = 0.0
        self.center_steer = 0.0

        # Parameters
        self.lin_step = 0.1
        self.max_v = 1.0
        self.steer_step = 5.0  # 5-degree step!

        # --- Key Repeat Filter Variables ---
        self.last_key = ''
        self.last_key_time = self.get_clock().now()

        # --- Actuator Variables ---
        self.actuator_cmd = 0.0
        self.last_actuator_time = self.get_clock().now()

        # Continuous Publishing Timer (20Hz)
        self.timer = self.create_timer(0.05, self.publish_state)

        self.get_logger().info(
            "Drive Controller Started. Modes: [h] Home, [n] Crab, "
            "[c] Pivot. Actuator: [1]/[2]. Streaming at 20Hz..."
        )

    def keyboard_callback(self, msg):
        key = msg.data.lower()
        now = self.get_clock().now()

        # --- Actuator Controls (1 = Push, 2 = Pull) ---
        if key == '1':
            self.actuator_cmd = 1.0
            self.last_actuator_time = now
            return
        elif key == '2':
            self.actuator_cmd = 2.0
            self.last_actuator_time = now
            return

        # --- High-Frequency Key Repeat Filter (W and S ONLY) ---
        if key in ['w', 's']:
            time_diff = (
                now - self.last_key_time
            ).nanoseconds / 1e9

            if key == self.last_key and time_diff < 0.2:
                self.last_key_time = now
                return

            self.last_key = key
            self.last_key_time = now
        else:
            self.last_key = key
            self.last_key_time = now

        # Drive Controls
        if key == 'w':
            self.vx = min(
                self.max_v,
                self.vx + self.lin_step
            )

        elif key == 's':
            self.vx = max(
                -self.max_v,
                self.vx - self.lin_step
            )

        # Steering Controls
        elif key == 'd' and self.mode == 'ackermann':
            self.center_steer = max(
                self.center_steer - self.steer_step,
                -45.0
            )

        elif key == 'a' and self.mode == 'ackermann':
            self.center_steer = min(
                self.center_steer + self.steer_step,
                45.0
            )

        # --- MODE SWITCHING COMMANDS ---
        elif key == 'h':
            self.mode = 'ackermann'
            self.vx = 0.0
            self.center_steer = 0.0

            self.get_logger().info(
                "\n--- Mode: ACKERMANN (Straight/Steer) ---"
            )

        elif key == 'n':
            self.mode = 'crab'
            self.vx = 0.0
            self.center_steer = 0.0

            self.get_logger().info(
                "\n--- Mode: 90 DEGREE (Crab Walk) ---"
            )

        elif key == 'c':
            self.mode = 'pivot'
            self.vx = 0.0
            self.center_steer = 0.0

            self.get_logger().info(
                "\n--- Mode: PIVOT (Spin in Place) ---"
            )

        # Emergency Stop
        elif key == ' ':
            self.vx = 0.0
            self.center_steer = 0.0
            self.last_key = ''

        elif key == 'q':
            self.vx = 0.0
            self.center_steer = 0.0
            self.publish_state()
            rclpy.shutdown()
            return

    def publish_state(self):

        # --- Actuator Auto-Stop Logic ---
        now = self.get_clock().now()

        if (
            now - self.last_actuator_time
        ).nanoseconds / 1e9 > 0.2:
            self.actuator_cmd = 0.0

        msg_angles = Float32MultiArray()
        msg_wheels = Float32MultiArray()

        # Dashboard display variables
        theta_left, theta_right = 0.0, 0.0

        # =====================================================
        # 1. CRAB MODE
        # =====================================================

        if self.mode == 'crab':
            angles = [
                90.0,
                90.0,
                90.0,
                90.0
            ]

            v_lf = -self.vx
            v_lr = -self.vx
            v_rf = self.vx
            v_rr = self.vx

        # =====================================================
        # 2. PIVOT MODE
        # =====================================================

        elif self.mode == 'pivot':
            angles = [
                45.0,
                -45.0,
                -45.0,
                45.0
            ]

            v_lf = -self.vx
            v_lr = -self.vx
            v_rf = -self.vx
            v_rr = -self.vx

        # =====================================================
        # 3. ACKERMANN MODE
        # =====================================================

        else:
            alpha = math.radians(
                self.center_steer
            )

            if abs(alpha) < 0.001:
                theta_left = 0.0
                theta_right = 0.0

            else:
                R = self.L / math.tan(alpha)

                theta_left = math.degrees(
                    math.atan(
                        self.L /
                        (R + self.W / 2.0)
                    )
                )

                theta_right = math.degrees(
                    math.atan(
                        self.L /
                        (R - self.W / 2.0)
                    )
                )

            angles = [
                theta_left,
                0.0,
                theta_right,
                0.0
            ]

            if abs(self.center_steer) < 0.001:
                v_lf = -self.vx
                v_lr = -self.vx
                v_rf = self.vx
                v_rr = self.vx

            else:
                alpha = math.radians(
                    abs(self.center_steer)
                )

                R = self.L / math.tan(alpha)

                r_fixed_inner = (
                    R - self.W / 2.0
                )

                r_fixed_outer = (
                    R + self.W / 2.0
                )

                r_steered_inner = math.sqrt(
                    r_fixed_inner ** 2 +
                    self.L ** 2
                )

                r_steered_outer = math.sqrt(
                    r_fixed_outer ** 2 +
                    self.L ** 2
                )

                v_base = abs(self.vx)

                omega = (
                    v_base /
                    r_steered_outer
                )

                v_of = v_base

                v_if = (
                    omega *
                    r_steered_inner
                )

                v_ob = (
                    omega *
                    r_fixed_outer
                )

                v_ib = (
                    omega *
                    r_fixed_inner
                )

                if self.center_steer > 0:
                    mag_lf = v_of
                    mag_lr = v_ob
                    mag_rf = v_if
                    mag_rr = v_ib

                else:
                    mag_lf = v_if
                    mag_lr = v_ib
                    mag_rf = v_of
                    mag_rr = v_ob

                direction = (
                    1.0
                    if self.vx >= 0
                    else -1.0
                )

                v_lf = (
                    -mag_lf *
                    direction
                )

                v_lr = (
                    -mag_lr *
                    direction
                )

                v_rf = (
                    mag_rf *
                    direction
                )

                v_rr = (
                    mag_rr *
                    direction
                )


        # =====================================================
        # PUBLISH TOPICS
        # =====================================================

        msg_angles.data = [
            float(a)
            for a in angles
        ]

        self.pub_angles.publish(
            msg_angles
        )

        msg_wheels.data = [
            float(v_lf),
            float(v_lr),
            float(v_rf),
            float(v_rr),
            float(self.actuator_cmd)
        ]

        self.pub_wheels.publish(
            msg_wheels
        )

        # Dashboard Feedback
        print(
            f"\rMode: {self.mode.upper():<9} | "
            f"Speed: {self.vx:5.1f} | "
            f"Input: {self.center_steer:5.1f}° | "
            f"W0(L): {angles[0]:5.1f}° | "
            f"W2(R): {angles[2]:5.1f}° | "
            f"Act: {int(self.actuator_cmd)}    ",
            end="",
            flush=True
        )


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
