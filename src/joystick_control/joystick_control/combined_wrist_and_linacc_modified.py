import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray,Float64
from getch import getch
from keymap import keytoState
from joymap import mapJoystickToAction

class JoyStickNode(Node):

    def __init__(self):
        super().__init__("joystick")

        self.joysub = self.create_subscription(Joy, "/joy", self.joycallback, 10)
        self.encoderDataSub = self.create_subscription(Int32MultiArray, "/arm_encoder_data", self.currStateProcess, 10)
        self.publisher = self.create_publisher(Int32MultiArray, "/arm_joint_states", 10)

        self.keyboardSpinner = self.create_timer(0.01, self.keyboardCallback)

        self.previousKey = None
        
    
    def joycallback(self, msg):
        self.buttons = msg.buttons
        self.axes = msg.axes

        joyArray = self.buttons[7:12]
        trgtState = mapJoystickToAction(joyArray)
        if trgtState is not None:
            toPublish = Int32MultiArray()
            toPublish.data = trgtState
            self.publisher.publish(toPublish)
            self.get_logger().info(f"Published State : {toPublish.data}")

    def currStateProcess(self, msg):
        self.upperActuatorState = msg.data[0]
        self.lowerActuatorState = msg.data[1]

    def keyboardCallback(self):
        key = getch()
        if key is not None and self.previousKey!=key:
            if key != 'q':
                self.data_array = keytoState(key)
                stateToPublish = Int32MultiArray()
                stateToPublish.data = self.data_array
                self.publisher.publish(stateToPublish)
                self.get_logger().info(f"Published state : {stateToPublish.data}")
            else:
                raise KeyboardInterrupt
    