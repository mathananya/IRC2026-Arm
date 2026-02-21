import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray, Int32
from joymap import mapJoystickToAction


class JOYButtons(Node):
    def __init__(self):
        super().__init__('joybuttonsnode')
        self.sub = self.create_subscription(Joy, '/joy', self.joycallback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, "arm_target_states", 10)
        self.pwmpub = self.create_publisher(Int32MultiArray, "arm_pwm_commands", 10)
        
        self.toggle_pub = self.create_publisher(Int32, "ik_toggle_state", 10)
        self.toggle_sub = self.create_subscription(Int32, "ik_toggle_state", self.toggle_callback, 10)
        self.IKToggle = 0      

    def toggle_callback(self, msg):
        self.IKToggle = msg.data
        self.get_logger().info(f"IK Toggle updated: {bool(self.IKToggle)}")

    def joycallback(self, msg):
        if(bool(self.IKToggle)):
            self.buttons = msg.buttons

            joyArray = self.buttons[7:12]
            
            trgtState = mapJoystickToAction(joyArray)
            if trgtState is not None:
                toPublish = Int32MultiArray()
                toPublish.data = trgtState
                self.publisher.publish(toPublish)
                self.get_logger().info(f"Published State : {toPublish.data}")
            
        

def main(args=None):
    rclpy.init(args=args)
    joystick_buttons = JOYButtons()
    
    try:
        rclpy.spin(joystick_buttons)
    except KeyboardInterrupt:
        pass
    finally:
        joystick_buttons.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
