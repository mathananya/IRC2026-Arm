import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray,Float64
from getch import getch
from keymap import keytoState
from joymap import mapJoystickToAction
from wrist_data_remap import wrist_map

class JoyStickNode(Node):

    def __init__(self):
        super().__init__("joystick")

        self.joysub = self.create_subscription(Joy, "/joy", self.joycallback, 10)
        self.encoderDataSub = self.create_subscription(Int32MultiArray, "/arm_encoder_data", self.currStateProcess, 10)
        self.publisher = self.create_publisher(Int32MultiArray, "arm_target_states", 10)
        self.publisher_wrist = self.create_publisher(Int32MultiArray, "arm_wrist_commands", 10)
        self.pwmpub = self.create_publisher(Int32MultiArray, "arm_pwm_commands", 10)

        self.keyboardSpinner = self.create_timer(0.01, self.keyboardCallback)

        self.previousKey = None
        
        self.upperActuatorState = 0
        self.lowerActuatorState = 0
        
        self.last_pressed = 0
        
        self.wrist_pwm_left = 0
        self.wrist_pwm_right = 0
        
    
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
            return
        
        x_flag = msg.buttons[5]
        y_flag = msg.buttons[3]
        
        if x_flag == 1 and y_flag == 0 and self.last_pressed != 3:
            self.last_pressed = 3
            if msg.axes[1] != 0:
                pwm_analog_value = msg.axes[1] / abs(msg.axes[1])
            else:
                pwm_analog_value = 0.0
            
            joystick_target = Int32MultiArray()
            bicep_target = int(self.upperActuatorState - (pwm_analog_value * 16 * 2.5))
            tricep_target = int(self.lowerActuatorState - (pwm_analog_value * 16 * 5.9))
            joystick_target.data = [bicep_target, tricep_target, 0, 0, 0]
            self.publisher.publish(joystick_target)
            self.get_logger().info(f"X-axis movement: {joystick_target.data}")
            return
        
        if x_flag == 0 and y_flag == 1 and self.last_pressed != 4:
            self.last_pressed = 4
            if msg.axes[1] != 0:
                pwm_analog_value = msg.axes[1] / abs(msg.axes[1])
            else:
                pwm_analog_value = 0.0
            
            joystick_target = Int32MultiArray()
            bicep_target = int(self.upperActuatorState + (pwm_analog_value * 16 * 2.5))
            tricep_target = int(self.lowerActuatorState + (pwm_analog_value * 16 * 5.9))
            joystick_target.data = [bicep_target, tricep_target, 0, 0, 0]
            self.publisher.publish(joystick_target)
            self.get_logger().info(f"Y-axis movement: {joystick_target.data}")
            return
        
        if x_flag == 0 and y_flag == 0:
            if self.last_pressed in [3, 4]:
                self.last_pressed = 0
        
        bicep_flag = msg.buttons[2]
        tricep_flag = msg.buttons[4]
        
        if bicep_flag == 1 and tricep_flag == 0 and self.last_pressed != 1:
            self.last_pressed = 1
            if msg.axes[1] != 0:
                pwm_analog_value = msg.axes[1] / abs(msg.axes[1])
            else:
                pwm_analog_value = 0.0
            
            joystick_target = Int32MultiArray()
            bicep_target = int(self.upperActuatorState + (pwm_analog_value * 40))
            joystick_target.data = [bicep_target, self.lowerActuatorState, 0, 0, 0]
            self.publisher.publish(joystick_target)
            self.get_logger().info(f"Bicep movement: {joystick_target.data}")
            return
        
        if bicep_flag == 0 and tricep_flag == 1 and self.last_pressed != 2:
            self.last_pressed = 2
            if msg.axes[1] != 0:
                pwm_analog_value = msg.axes[1] / abs(msg.axes[1])
            else:
                pwm_analog_value = 0.0
            
            joystick_target = Int32MultiArray()
            tricep_target = int(self.lowerActuatorState + (pwm_analog_value * 40))
            joystick_target.data = [self.upperActuatorState, tricep_target, 0, 0, 0]
            self.publisher.publish(joystick_target)
            self.get_logger().info(f"Tricep movement: {joystick_target.data}")
            return
        
        if bicep_flag == 0 and tricep_flag == 0:
            if self.last_pressed in [1, 2]:
                self.last_pressed = 0
        
        if msg.buttons[6] == 1:
            pwm_publish_wrist = Int32MultiArray()
            
            wrist_data = msg.axes[-2:]
            gripper_data = msg.axes[3]
            gripper_pwm = int(gripper_data * 100)
            
            res = wrist_map(wrist_data)
            if res is not None:
                self.wrist_pwm_left, self.wrist_pwm_right = res[0], res[1]
            
            pwm_publish_wrist.data = [0, 0, self.wrist_pwm_left, self.wrist_pwm_right, gripper_pwm]
            self.publisher_wrist.publish(pwm_publish_wrist)
            self.get_logger().info(f"Wrist PWM: {pwm_publish_wrist.data}")

    def currStateProcess(self, msg):
        self.upperActuatorState = msg.data[0]
        self.lowerActuatorState = msg.data[1]

    def keyboardCallback(self):
        key = getch()
        if key is not None :
            if key != 'q':
                self.data_array = keytoState(key)
                stateToPublish = Int32MultiArray()
                stateToPublish.data = self.data_array
                # self.publisher_wrist.publish(stateToPublish)
                self.pwmpub.publish(stateToPublish)
                self.get_logger().info(f"Published state : {stateToPublish.data}")
                self.previousKey = key
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
