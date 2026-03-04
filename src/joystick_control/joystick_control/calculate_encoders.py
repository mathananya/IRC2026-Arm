#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
import threading

def angle_upper_to_encoder(angle_degrees):
    encoder_value = -34.84286 * angle_degrees + 3541.92857
    return int(encoder_value)

def angle_lower_to_encoder(angle_degrees):
    encoder_value = 17.51429 * angle_degrees + 1534.28571
    return int(encoder_value)

class TargetEncoderPublisher(Node):
    def __init__(self):
        super().__init__('target_encoder_publisher')
        self.ikpub = self.create_publisher(Int32, 'ik_toggle_state', 10)
        self.publisher_ = self.create_publisher(Int32MultiArray, 'arm_target_states', 10)
        self.get_logger().info('Target Encoder Publisher started. Type "go <lower> <upper>" to send commands.')

        self.IKOnMsg = Int32()
        self.IKOnMsg.data = 1
        self.ikpub.publish(self.IKOnMsg)

    def publish_target(self, lower_enc, upper_enc):
        msg = Int32MultiArray()
        # [lower_enc, upper_enc, 0, 0, 0]
        msg.data = [lower_enc, upper_enc, 0, 0, 0]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published target encoders: {msg.data}')

def input_thread(node):
    while rclpy.ok():
        try:
            user_input = input("Enter command 'go <lower> <upper>' (or 'q' to quit): ")
            
            if user_input.strip().lower() == 'q':
                node.get_logger().info("Shutting down...")
                rclpy.shutdown()
                break
                
            parts = user_input.strip().split()
            if len(parts) != 3 or parts[0].lower() != 'go':
                print("Invalid command. Use format: go <lower> <upper>")
                continue
                
            lower_str = parts[1]
            upper_str = parts[2]
            
            lower_angle = float(lower_str.strip())
            upper_angle = float(upper_str.strip())

            lower_encoder = angle_lower_to_encoder(lower_angle)
            upper_encoder = angle_upper_to_encoder(upper_angle)

            print(f"Calculated Encoders -> Lower: {lower_encoder}, Upper: {upper_encoder}")
            print("-" * 20)
            
            node.publish_target(lower_encoder, upper_encoder)
            
        except ValueError:
            print("Invalid input. Please enter valid numbers for angles (e.g., 'go 0 90').")
        except EOFError:
            break
        except Exception as e:
            print(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TargetEncoderPublisher()
    
    # Run user input loop in a separate thread so node can still log
    thread = threading.Thread(target=input_thread, args=(node,))
    thread.daemon = True
    thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
