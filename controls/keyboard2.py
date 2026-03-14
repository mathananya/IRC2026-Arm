#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from getch import getch

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.pub_keyboard = self.create_publisher(String, '/keyBoard', 10)
        
        # We start the loop in a timer so the node initializes properly first
        self.timer = self.create_timer(0.5, self.run_keyboard_listener)
        self.get_logger().info("Keyboard Node Started. Press W/A/S/D or SPACE.")

    def run_keyboard_listener(self):
        self.timer.cancel() # Stop the timer so we only run this loop once
        try:
            while rclpy.ok():
                key = getch()
                # Publish the key if it's one of our controls
                if key.lower() in ['w', 'a', 's', 'd', ' ', 'q','n','c','h']:
                    msg = String()
                    msg.data = key.lower()
                    self.pub_keyboard.publish(msg)
                    
                    if key.lower() == 'q' or key == '\x03': # Quit
                        break
        except Exception as e:
            print(f"Error: {e}")
        finally:
            print("\nShutting down keyboard node.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
