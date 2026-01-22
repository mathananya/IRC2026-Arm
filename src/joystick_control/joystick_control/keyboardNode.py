import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading
from getch import getch
from keymap import keytoState

class KeyboardNode(Node):
    
    def __init__(self):
        super().__init__('keyboard')
        self.pwmpub = self.create_publisher(Int32MultiArray, '/arm_pwm_commands', 10)
        self.running = True
        self.get_logger().info("Keyboard control node has successfully started!")
        self.get_logger().info("Press keys to control, 'q' to quit")
        
        # Start keyboard thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        
    def keyboard_loop(self):
        """Runs in separate thread to handle blocking getch()"""
        while self.running:
            try:
                key = getch()
                if key is not None:
                    if key == 'q':
                        self.get_logger().info("Quit key pressed, shutting down...")
                        self.running = False
                        break
                    
                    # Publish the key command
                    data_array = keytoState(key)
                    state_to_publish = Int32MultiArray()
                    state_to_publish.data = data_array
                    self.pwmpub.publish(state_to_publish)
                    self.get_logger().info(f"Key: '{key}' -> State: {state_to_publish.data}")
                    
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                self.running = False
                break
    
    def is_running(self):
        """Check if node should continue running"""
        return self.running
    
    def destroy_node(self):
        """Clean up"""
        self.running = False
        if self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    keyboard_node = KeyboardNode()
    
    try:
        while rclpy.ok() and keyboard_node.is_running():
            rclpy.spin_once(keyboard_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        keyboard_node.get_logger().info("Keyboard interrupt received")
    finally:
        keyboard_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()