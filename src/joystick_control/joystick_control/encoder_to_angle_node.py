#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray

class EncoderToAngleNode(Node):
    def __init__(self):
        super().__init__('encoder_to_angle_node')
        
        # Subscriber to encoder data
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'arm_encoder_data',
            self.encoder_callback,
            10)
            
        # Publisher for angle data
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'arm_angles',
            10)
            
        self.get_logger().info('Encoder to Angle Node started.')

    def encoder_to_angle_lower(self, encoder_value):
        # Uses equation: Angle = 0.05689 * Encoder - 87.22924
        angle_degrees = 0.05689 * encoder_value - 87.22924
        return float(angle_degrees)

    def encoder_to_angle_upper(self, encoder_value):
        # Uses equation: Angle = -0.02863 * Encoder + 101.58735
        angle_degrees = -0.02863 * encoder_value + 101.58735
        return float(angle_degrees)

    def encoder_callback(self, msg):
        if len(msg.data) >= 2:
            lower_encoder = msg.data[0]
            upper_encoder = msg.data[1]
            
            lower_angle = self.encoder_to_angle_lower(lower_encoder)
            upper_angle = self.encoder_to_angle_upper(upper_encoder)
            
            angle_msg = Float32MultiArray()
            # Publish as [lower_angle, upper_angle]
            angle_msg.data = [lower_angle, upper_angle]
            self.publisher.publish(angle_msg)
            
            self.get_logger().info(
                f'Encoders -> [Lower: {lower_encoder}, Upper: {upper_encoder}] '
                f'Angles -> [Lower: {lower_angle:.2f}, Upper: {upper_angle:.2f}]')
        else:
            self.get_logger().warning('Received encoder data array with less than 2 elements.')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
