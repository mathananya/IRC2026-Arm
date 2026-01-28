import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool, Int32
from pid2 import PIDController


class ArmControllerIntegrated(Node):
    def __init__(self):
        super().__init__('arm_controller_integrated')
         
        Kp1 = 6.5
        Ki1 = 0.01
        Kd1 = 0.02


        Kp2 = 5.0
        Ki2 = 0.01
        Kd2 = 0.02
        # Kp = 0.7
        # Ki = 0.01
        # Kd = 0.02


        #flag here is used to indicate opposite direction
        self.lower_pid = PIDController(Kp1, Ki1, Kd1, integral_max=100, margin_of_error=5, flag = 1)
        self.upper_pid = PIDController(Kp2, Ki2, Kd2, integral_max=100, margin_of_error=5)
        
        
        self.encoder_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_encoder_data', self.encoder_callback, 10)
        self.target_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_target_states', self.target_callback, 10)
        
        
        self.pwm_publisher = self.create_publisher(Int32MultiArray, 'arm_pwm_commands', 10)
        self.busy_publisher = self.create_publisher(Bool, 'arm_pid_busy', 10)
        self.togglesub = self.create_subscription(Int32, 'ik_toggle_state', self.IKCallback, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.current_states = [0.0, 0.0]
        self.target_states = [0.0, 0.0]
        self.is_busy = False
        self.lower_pwm = 0
        self.upper_pwm = 0
        self.start_PID1 = False
        self.start_PID2 = False 
        
        self.wrist_pwm = [0, 0, 0]
        
        self.upper_limit_flag = False
        self.lower_limit_flag = False
        self.stop_decision = True
        # self.limit_margin = 15
        self.PIDMargin = 5
        self.PWMmax = 250
        # self.prevIKToggle = None
        self.prevBusy = None
        self.IKToggle = False
        
        self.get_logger().info('Integrated Arm Controller Started')

    def encoder_callback(self, msg):
        self.current_states = msg.data[:2]
        
    def target_callback(self, msg):
        self.target_states = msg.data[:2]
        self.get_logger().info(f'Target received: {self.target_states}')
        self.start_PID1 = True
        self.start_PID2 = True
        self.is_busy = True

    def IKCallback(self, msg):
        self.IKToggle = msg.data
        print(f"IK Toggle callback activated in controller, state : {self.IKToggle}")
    
    def control_loop(self):
        # if(self.IKToggle):
        if(len(self.target_states) > 0):        
            if self.start_PID1 == True or self.start_PID2 == True:

                if((abs(self.current_states[0] - self.target_states[0]) < self.PIDMargin)):
                    self.get_logger().info("Lower one switched off!")
                    self.start_PID1 = False
                    self.lower_pid.updateError(0)
                    self.lower_pid.updateIntegral(0)

                if((abs(self.current_states[1] - self.target_states[1]) < self.PIDMargin)):
                    self.get_logger().info("Upper one switched off!")
                    self.start_PID2 = False
                    self.upper_pid.updateError(0)
                    self.upper_pid.updateIntegral(0)

                lower_current_value, upper_current_value = self.current_states
                lower_target_value, upper_target_value = self.target_states

                if(self.start_PID1==False):
                    self.lower_pwm = 0
                else:
                    self.lower_pwm = self.lower_pid.update(lower_current_value, lower_target_value)

                if(self.start_PID2==False):
                    self.upper_pwm = 0
                else:
                    self.upper_pwm = self.upper_pid.update(upper_current_value, upper_target_value)
                self.upper_pwm = min(max(self.upper_pwm, -self.PWMmax), self.PWMmax)
                self.lower_pwm = min(max(self.lower_pwm, -self.PWMmax), self.PWMmax)

                pwm_values = Int32MultiArray()
                pwm_values.data = [int(self.lower_pwm), int(self.upper_pwm), 0, 0, 0]

                self.pwm_publisher.publish(pwm_values)
                
                if (self.start_PID1 == False and self.start_PID2 == False):
                    if ((self.lower_pwm | self.upper_pwm) == 0):
                        self.is_busy = False
                        self.get_logger().info('Arm not busy - ready for next target')
                    else:
                        self.is_busy = True
                if (self.IKToggle == 0):
                    self.is_busy = False
                    self.target_states = []
                    print(f"Target states cleared : {self.target_states}")
                    self.start_PID1 = False
                    self.start_PID2 = False
                
                
                if(self.is_busy!= self.prevBusy):
                    busy_msg = Bool()
                    busy_msg.data = self.is_busy
                    self.busy_publisher.publish(busy_msg)
                    self.prevBusy = self.is_busy
            
                self.get_logger().info(
                    f'Lower: {lower_current_value}->{lower_target_value} PWM:{self.lower_pwm} | '
                    f'Upper: {upper_current_value}->{upper_target_value} PWM:{self.upper_pwm} | Busy:{self.is_busy}'
                )

    def shutdown(self):
        self.get_logger().info("Shutting down Integrated Arm Controller...")


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerIntegrated()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
