import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray,Float64
from getch import getch
from keymap import keytoState
from joymap import mapJoystickToAction
from joymap import mapJoyAxes
import threading
class JoyStickNode(Node):

    def __init__(self):
        super().__init__("joystick")

        self.joysub = self.create_subscription(Joy, "/joy", self.joycallback, 10)
        # self.encoderDataSub = self.create_subscription(Int32MultiArray, "arm_encoder_data", self.currStateProcess, 10)
        self.publisher = self.create_publisher(Int32MultiArray, "arm_target_states", 10)
        # self.publisher_wrist = self.create_publisher(Int32MultiArray, "arm_wrist_commands", 10)
        self.pwmpub = self.create_publisher(Int32MultiArray, "arm_pwm_commands", 10)

        self.keyboardSpinnerTimer = self.create_timer(0.01, self.keyboardCallback)


        self.previousKey = None
        
        self.upperActuatorState = 0
        self.lowerActuatorState = 0
        
        self.last_pressed = 0
        
        self.wrist_pwm_left = 0
        self.wrist_pwm_right = 0
        
    
    def joycallback(self, msg):
        print("hi")
        self.buttons = msg.buttons
        self.axes = msg.axes

        joyArray = self.buttons[7:12]
        pwmPubVal = mapJoyAxes(self.axes)

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
           

    def keyboardCallback(self):
        key = getch()
        if key is not None :
            if key != 'q':
                self.data_array = keytoState(key)
                stateToPublish = Int32MultiArray()
                stateToPublish.data = self.data_array
                self.pwmpub.publish(stateToPublish)
                self.get_logger().info(f"Published state : {stateToPublish.data}")
                # self.previousKey = key
            else:
                raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoyStickNode()
    
    try:
        rclpy.spin(joystick_node)
    except KeyboardInterrupt:
        pass
    finally:
        joystick_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
