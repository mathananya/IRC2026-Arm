import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from joymap import mapJoystickToAction

class JOYButtons(Node):
    def __init__(self):
        super().__init__('joynode')
        self.sub = self.create_subscription(Joy, '/joy', self.joycallback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, "arm_target_states", 10)
        self.pwmpub = self.create_publisher(Int32MultiArray, "arm_pwm_commands", 10)

    def joycallback(self, msg):

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
