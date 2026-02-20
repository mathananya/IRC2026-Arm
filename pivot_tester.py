import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import tty
import termios

# This function reads a single keypress instantly without needing 'Enter'
def get_keystroke():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class PivotTester(Node):
    def __init__(self):
        super().__init__('crab_steer_tester')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'target_angles', 10)

    def publish_angles(self, angles):
        msg = Float32MultiArray()
        parsed_angles = [float(a) for a in angles]
        
        # --- Keeps your 4th wheel (index 3) inversion logic ---
        parsed_angles[3] = 360.0 - parsed_angles[3]
        
        msg.data = parsed_angles
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PivotTester()
    
    current_angle = 0.0  # Master angle for all wheels
    step_size = 1.0      # Increment/Decrement by 1 degree
    
    print("=========================================")
    print("   ESP32 Instant Crab-Steer Tester       ")
    print("=========================================")
    print("Press 'D' to INCREASE angle by 1°")
    print("Press 'A' to DECREASE angle by 1°")
    print("Press 'Q' to quit.")
    print("=========================================\n")
    
    try:
        # Publish initial 0 position at startup
        node.publish_angles([current_angle, current_angle, current_angle, current_angle])
        print(f"\rCurrent Angle: {current_angle:3.1f}°  ", end="", flush=True)
        
        while rclpy.ok():
            char = get_keystroke()
            
            if char.lower() == 'q':
                break
            elif char.lower() == 'd':
                # Increase and wrap around at 360
                current_angle = (current_angle + step_size) % 360.0
            elif char.lower() == 'a':
                # Decrease and wrap around below 0
                current_angle = (current_angle - step_size) % 360.0
            else:
                continue # Ignore any other keys
            
            # Print the angle on the same line so it doesn't flood the terminal
            print(f"\rCurrent Angle: {current_angle:3.1f}°  ", end="", flush=True)
            
            # Send the new angle to all 4 wheels
            node.publish_angles([current_angle, current_angle, current_angle, current_angle])
            
    except KeyboardInterrupt:
        pass
        
    finally:
        print("\n\nExiting...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
