import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from joymap import mapJoyAxes
from poseToPWM import pose_to_encoder, encoder_to_pose

X_Z_STEP = 10

class JoyAxes(Node):
    def __init__(self):
        super().__init__('joynode')
        self.sub = self.create_subscription(Joy, '/joy', self.joycallback, 10)
        self.encsub = self.create_subscription(Int32MultiArray, '/arm_encoder_data', self.encCallback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, "arm_target_states", 10)
        self.pwmpub = self.create_publisher(Int32MultiArray, "arm_pwm_commands", 10)

    def encCallback(self, msg):
        global lowerEnc
        global upperEnc
        lowerEnc = msg.data[0]
        upperEnc = msg.data[1]

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

        if(self.buttons[0] != 1):
            pwmPubVal = mapJoyAxes(joyAx)
            pwmMsg = Int32MultiArray()
            pwmMsg.data = pwmPubVal
            self.pwmpub.publish(pwmMsg)
            self.get_logger().info(f"Published axes message : {pwmMsg.data}")
        else:
            initialx, initialz = encoder_to_pose(lower_encoder = lowerEnc, upper_encoder = upperEnc)
            finalx, finalz = 0, 0
            if(joyAx[1] == 1):
                finalx = initialx + X_Z_STEP
                finalz = initialz
            elif(joyAx[1] == -1):
                finalx = initialx - X_Z_STEP
                finalz = initialz
            elif(joyAx[0] == 1):
                finalx = initialx
                finalz = initialz + X_Z_STEP
            elif(joyAx[0] == -1):
                finalx = initialx
                finalz = initialz - X_Z_STEP
            
            finalUpperEnc, finalLowerEnc = pose_to_encoder(finalx, finalz)
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
