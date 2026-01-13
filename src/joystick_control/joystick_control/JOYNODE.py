import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from joymap import mapJoystickToAction, mapJoyAxes

class JOYNODE(Node):
    def __init__(self):
        super().__init__('joynode')
        self.sub = self.create_subscription(Joy, '/joy', self.joycallback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, "arm_target_states", 10)
        self.pwmpub = self.create_publisher(Int32MultiArray, "arm_pwm_commands", 10)

    def joycallback(self, msg):

        self.buttons = msg.buttons
        self.axes = msg.axes

        joyAx = []
        for val in self.axes:
            if(abs(val) >= 0.5):
                if(val > 0):
                    joyAx.append(1)
                elif(val < 0):
                    joyAx.append(-1)
            else:
                joyAx.append(0)

        joyArray = self.buttons[7:12]
        pwmPubVal = mapJoyAxes(joyAx)

        pwmMsg = Int32MultiArray()
        pwmMsg.data = pwmPubVal
        self.pwmpub.publish(pwmMsg)
        self.get_logger().info(f"Published axes message : {pwmMsg.data}")
        
        trgtState = mapJoystickToAction(joyArray)
        if trgtState is not None:
            toPublish = Int32MultiArray()
            toPublish.data = trgtState
            self.publisher.publish(toPublish)
            self.get_logger().info(f"Published State : {toPublish.data}")
        
        

def main(args=None):
    rclpy.init(args=args)
    joystick_node = JOYNODE()
    
    try:
        rclpy.spin(joystick_node)
    except KeyboardInterrupt:
        pass
    finally:
        joystick_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
