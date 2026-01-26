import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from joymap import mapJoyAxes
from poseToPWM import pose_to_encoder, encoder_to_pose

X_Z_STEP = 100
GRIPPER_STEP = 200
WRIST_STEP = 50

class JoyAxes(Node):
    def __init__(self):
        super().__init__('joynode')
        self.sub = self.create_subscription(Joy, '/joy', self.joycallback, 10)
        self.encsub = self.create_subscription(Int32MultiArray, '/arm_encoder_data', self.encCallback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, "arm_target_states", 10)
        self.pwmpub = self.create_publisher(Int32MultiArray, "arm_pwm_commands", 10)
        self.lowerEnc = None
        self.upperEnc = None

    def encCallback(self, msg):
        self.lowerEnc = msg.data[0]
        self.upperEnc = msg.data[1]
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
        
        # if(joyAx[3] == 1):
        #     pwmPubVal = [0,0,0,0,GRIPPER_STEP]
        #     pwmMsg = Int32MultiArray()
        #     pwmMsg.data = pwmPubVal
        #     self.pwmpub.publish(pwmMsg)
        #     self.get_logger().info(f"Gripper moving by {GRIPPER_STEP} positive!")
        # elif(joyAx[3] == -1):
        #     pwmPubVal = [0,0,0,0,-GRIPPER_STEP]
        #     pwmMsg = Int32MultiArray()
        #     pwmMsg.data = pwmPubVal
        #     self.pwmpub.publish(pwmMsg)
        #     self.get_logger().info(f"Gripper moving by {GRIPPER_STEP} negative!")
        
        if(joyAx[3] == 1):
            if(joyAx[5] == 1):
                if(self.buttons[0] == 1):
                    pwmPubVal = [0,0,0,0,0]
                else:
                    pwmPubVal = [0,0,WRIST_STEP,WRIST_STEP,0]
                    self.get_logger().info(f"Wrist moving same direction by {WRIST_STEP} positive")
                pwmMsg = Int32MultiArray()
                pwmMsg.data = pwmPubVal
                self.pwmpub.publish(pwmMsg)
                
            elif(joyAx[5] == -1):
                if(self.buttons[0] == 1):
                    pwmPubVal = [0,0,0,0,0]
                else:
                    pwmPubVal = [0,0,-WRIST_STEP,-WRIST_STEP,0]
                    self.get_logger().info(f"Wrist moving same direction by {WRIST_STEP} negative")
                pwmMsg = Int32MultiArray()
                pwmMsg.data = pwmPubVal
                self.pwmpub.publish(pwmMsg)
            elif(joyAx[4] == 1):
                if(self.buttons[0] == 1):
                    pwmPubVal = [0,0,0,0,0]
                else:
                    pwmPubVal = [0,0,WRIST_STEP,-WRIST_STEP,0]
                    self.get_logger().info(f"Wrist moving opposite direction by {WRIST_STEP} positive")
                pwmMsg = Int32MultiArray()
                pwmMsg.data = pwmPubVal
                self.pwmpub.publish(pwmMsg)
            elif(joyAx[4] == -1):
                if(self.buttons[0] == -1):
                    pwmPubVal = [0,0,0,0,0]
                else:
                    pwmPubVal = [0,0,-WRIST_STEP,WRIST_STEP,0]
                    self.get_logger().info(f"Wrist moving opposite direction by {WRIST_STEP} negative")
                pwmMsg = Int32MultiArray()
                pwmMsg.data = pwmPubVal
                self.pwmpub.publish(pwmMsg)
            elif(joyAx[1] == 1):
                if(self.buttons[0] == 1):
                    pwmPubVal = [0,0,0,0,0]
                else:
                    pwmPubVal = [0,0,0,0,GRIPPER_STEP]
                pwmMsg = Int32MultiArray()
                pwmMsg.data = pwmPubVal
                self.pwmpub.publish(pwmMsg)
                self.get_logger().info(f"Gripper moving +{GRIPPER_STEP}")
            elif(joyAx[1] == -1):
                if(self.buttons[0] == 1):
                    pwmPubVal = [0,0,0,0,0]
                else:
                    pwmPubVal = [0,0,0,0,-GRIPPER_STEP]
                pwmMsg = Int32MultiArray()
                pwmMsg.data = pwmPubVal
                self.pwmpub.publish(pwmMsg)
                self.get_logger().info(f"Gripper moving by -{GRIPPER_STEP}")
        if(joyAx[3] == -1):
            pwmPubVal = mapJoyAxes(joyAx)
            if(not (pwmPubVal) == [0,0,0,0,0]):
                pwmMsg = Int32MultiArray()
                pwmMsg.data = pwmPubVal
                self.pwmpub.publish(pwmMsg)
                self.get_logger().info(f"Published axes message : {pwmMsg.data}")
        if(self.lowerEnc is not None and self.upperEnc is not None):
            initialx, initialz = encoder_to_pose(lower_encoder = self.lowerEnc, upper_encoder = self.upperEnc)
        finalx, finalz = None, None
        if(self.buttons[4] == 1):
            finalx = initialx + X_Z_STEP
            finalz = initialz
        elif(self.buttons[2] == 1):
            finalx = initialx - X_Z_STEP
            finalz = initialz
        elif(self.buttons[5] == 1):
            finalx = initialx
            finalz = initialz + X_Z_STEP
        elif(self.buttons[3] == 1):
            finalx = initialx
            finalz = initialz - X_Z_STEP
        if(finalx is not None and finalz is not None):    
            finalUpperEnc, finalLowerEnc = pose_to_encoder(x = finalx, z = finalz)
            targetState = [finalLowerEnc,finalUpperEnc,0,0,0]
            targetMsg = Int32MultiArray()
            targetMsg.data = targetState
            self.publisher.publish(targetMsg)
            self.get_logger().info(f"Published Target State : {targetMsg.data}")
        
        

def main(args=None):
    rclpy.init(args=args)
    joystickaxes = JoyAxes()
    
    try:
        rclpy.spin(joystickaxes)
    except KeyboardInterrupt:
        pass
    finally:
        joystickaxes.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
