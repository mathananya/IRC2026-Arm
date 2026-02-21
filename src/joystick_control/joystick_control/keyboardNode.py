import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from getch import getch
from keymap import keytoState

class keyboardNode(Node):
    
    def __init__(self):
        super().__init__('keyboard')

        self.pwmpub = self.create_publisher(Int32MultiArray, '/arm_pwm_commands', 10)
        self.keyboardSpinnerTimer = self.create_timer(0.01, self.keyboardCallback)
        self.get_logger().info("Keyboard control node has successfully started!")
        
    def keyboardCallback(self):
        key = getch()
        if key is not None :
            if key != 'q':
                self.data_array = keytoState(key)
                stateToPublish = Int32MultiArray()
                stateToPublish.data = self.data_array
                self.pwmpub.publish(stateToPublish)
                self.get_logger().info(f"Published state : {stateToPublish.data}")
            else:
                raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    keyboardn = keyboardNode()
    
    
    try:
        rclpy.spin(keyboardn)
    except KeyboardInterrupt:
        pass
    finally:
        keyboardn.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
