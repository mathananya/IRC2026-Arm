import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray, Int32, Bool
from joymap import mapJoyAxes
from poseToPWM import pose_to_encoder, encoder_to_pose
from collections import deque


X_Z_STEP = 100
GRIPPER_STEP = 200
LEFT_WRIST_STEP = 100
RIGHT_WRIST_STEP = 150
WRIST_STEP = 100


class JoyAxes(Node):
    def __init__(self):
        super().__init__('joyaxesnode')
        self.sub = self.create_subscription(Joy, '/joy', self.joycallback, 10)
        self.encsub = self.create_subscription(Int32MultiArray, '/arm_encoder_data', self.encCallback, 10)
        self.publisher = self.create_publisher(Int32MultiArray, "arm_target_states", 10)
        self.pwmpub = self.create_publisher(Int32MultiArray, "arm_pwm_commands", 10)
        
        self.toggle_pub = self.create_publisher(Int32, "ik_toggle_state", 10)
        self.toggle_sub = self.create_subscription(Int32, "ik_toggle_state", self.toggle_callback, 10)
        self.IKToggle = 0
        
        # Target queue and busy flag for IK mode
        self.target_queue = deque(maxlen=3)
        self.busy_sub = self.create_subscription(Bool, "arm_pid_busy", self.busy_callback, 10)
        self.arm_busy = False
        
        self.lowerEnc = None
        self.upperEnc = None
        self.prevPWMVal = None
        self.prevToggleButton = 0
        # self.prevIKToggle = None
    def encCallback(self, msg):
        self.lowerEnc = msg.data[0]
        self.upperEnc = msg.data[1]
    
    def toggle_callback(self, msg):
        self.IKToggle = msg.data
        self.get_logger().info(f"IK Toggle updated: {bool(self.IKToggle)}")
    
    def busy_callback(self, msg):
        self.arm_busy = msg.data
        self.get_logger().info(f"Arm busy status: {self.arm_busy}")
        # Try to process queue when arm becomes not busy
        if not self.arm_busy:
            self.process_target_queue()
    
    def process_target_queue(self):
        """Send next target from queue if arm is not busy"""
        if not self.arm_busy and len(self.target_queue) > 0:
            target_state = self.target_queue.popleft()
            target_msg = Int32MultiArray()
            target_msg.data = target_state
            self.publisher.publish(target_msg)
            self.get_logger().info(f"Published queued target: {target_msg.data} | Queue length: {len(self.target_queue)}")
    
    def joycallback(self, msg):
        self.buttons = msg.buttons
        self.axes = msg.axes

        # TODO: ADD YOUR TOGGLE BUTTON LOGIC HERE
        # Example: if self.buttons[YOUR_BUTTON_INDEX] == 1:
        if(self.buttons[1] == 1 and self.buttons[1] != self.prevToggleButton):
            self.IKToggle = self.IKToggle ^ 1 
            toggle_msg = Int32()
            toggle_msg.data = self.IKToggle
            self.toggle_pub.publish(toggle_msg)
            # self.get_logger().info(f"IK Mode Toggled: {bool(self.IKToggle)}")
            if(self.IKToggle == 0):
                self.target_queue.clear()
                self.arm_busy = False
                self.get_logger().info(f"Queue droppped, back to normal mode! Busy flag set to {self.arm_busy}")

            # self.prevIKToggle = self.IKToggle
        
        # TODO: ADD EMERGENCY STOP BUTTON HERE
        # Example: if self.buttons[YOUR_EMERGENCY_BUTTON_INDEX] == 1:
        # elif(self.buttons[1] == 1 and self.IKToggle == 1 and self.buttons[1] != self.prevToggleButton):
        #     self.target_queue.clear()
        #     self.IKToggle = 0
        #     toggle_msg = Int32()
        #     toggle_msg.data = self.IKToggle
        #     self.toggle_pub.publish(toggle_msg)
        #     self.get_logger().info(f"Queue cleared, IK mode disabled!")
            # self.prevIKToggle = self.IKToggle
        self.prevToggleButton = self.buttons[1]     
        joyAx = []
        for val in self.axes:
            if(abs(val) >= 0.5):
                if(val > 0):
                    joyAx.append(1)
                elif(val < 0):
                    joyAx.append(-1)
            else:
                joyAx.append(0)
        
        if(not bool(self.IKToggle)):
            pwmPubVal = [0,0,0,0,0]
            if(joyAx[3] == 1):
                if(joyAx[5] == 1):
                    if(self.buttons[0] == 1):
                        pwmPubVal = [0,0,0,0,0]
                    else:
                        pwmPubVal = [0,0,RIGHT_WRIST_STEP,LEFT_WRIST_STEP,0]
                        self.get_logger().info(f"Wrist moving same direction by {WRIST_STEP} positive")
                elif(joyAx[5] == -1):
                    if(self.buttons[0] == 1):
                        pwmPubVal = [0,0,0,0,0]
                    else:
                        pwmPubVal = [0,0,-RIGHT_WRIST_STEP,-LEFT_WRIST_STEP,0]
                        self.get_logger().info(f"Wrist moving same direction by {WRIST_STEP} negative")
                elif(joyAx[4] == 1):
                    if(self.buttons[0] == 1):
                        pwmPubVal = [0,0,0,0,0]
                    else:
                        pwmPubVal = [0,0,RIGHT_WRIST_STEP,-LEFT_WRIST_STEP,0]
                        self.get_logger().info(f"Wrist moving opposite direction by {WRIST_STEP} positive")
                elif(joyAx[4] == -1):
                    if(self.buttons[0] == 1):
                        pwmPubVal = [0,0,0,0,0]
                    else:
                        pwmPubVal = [0,0,-RIGHT_WRIST_STEP,LEFT_WRIST_STEP,0]
                        self.get_logger().info(f"Wrist moving opposite direction by {WRIST_STEP} negative")
                elif(joyAx[1] == 1):
                    if(self.buttons[0] == 1):
                        pwmPubVal = [0,0,0,0,0]
                    else:
                        pwmPubVal = [0,0,0,0,GRIPPER_STEP]
                    self.get_logger().info(f"Gripper moving +{GRIPPER_STEP}")
                elif(joyAx[1] == -1):
                    if(self.buttons[0] == 1):
                        pwmPubVal = [0,0,0,0,0]
                    else:
                        pwmPubVal = [0,0,0,0,-GRIPPER_STEP]
                    self.get_logger().info(f"Gripper moving by -{GRIPPER_STEP}")
               
            if(joyAx[3] == -1):
               pwmPubVal = mapJoyAxes(joyAx)
               self.get_logger().info(f"Published axes message : {pwmPubVal}")
            pwmMsg = Int32MultiArray()
            pwmMsg.data = pwmPubVal
            self.pwmpub.publish(pwmMsg)      
        if(bool(self.IKToggle)):
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
                    targetState = [finalLowerEnc, finalUpperEnc, 0, 0, 0]
                    
                    # Add to queue instead of publishing immediately
                    self.target_queue.append(targetState)
                    self.get_logger().info(f"Target added to queue: {targetState} | Queue length: {len(self.target_queue)}")
                    
                    # Try to process immediately if arm not busy
                    self.process_target_queue()
        
        

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
