import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from pid import PIDController
import time


class ArmControllerIntegrated(Node):
    def __init__(self):
        super().__init__('arm_controller_integrated')
        
        
        Kp = 15.0
        Ki = 10.0
        Kd = 0.02
        self.upper_pid = PIDController(Kp, Ki, Kd, integral_max=10, integral_min=-10, margin_of_error=3)
        self.lower_pid = PIDController(Kp, Ki, Kd, integral_max=10, integral_min=-10, margin_of_error=3)
        
        
        self.encoder_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_encoder_data', self.encoder_callback, 10)
        self.target_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_target_states', self.target_callback, 10)
        self.wrist_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_wrist_commands', self.wrist_callback, 10)
        
        
        self.pwm_publisher = self.create_publisher(Int32MultiArray, 'arm_pwm_commands', 10)
        
        
        self.timer = self.create_timer(0.3, self.control_loop)
        
        self.current_states = [0.0, 0.0]
        self.target_states = [0.0, 0.0]
        self.upper_pwm = 0
        self.lower_pwm = 0
        self.start_PID = False
        
        self.wrist_pwm = [0, 0, 0]
        
        self.upper_limit_flag = False
        self.lower_limit_flag = False
        self.stop_decision = True
        self.limit_margin = 15
        
        self.get_logger().info('Integrated Arm Controller Started')

    def encoder_callback(self, msg):
        self.current_states = msg.data[:2]
        
    def target_callback(self, msg):
        self.target_states = msg.data[:2]
        self.get_logger().info(f'Target received: {self.target_states}')
        self.start_PID = True
        self.stop_decision = True
        
    def wrist_callback(self, msg):
        self.wrist_pwm = msg.data[2:5]
        self.get_logger().info(f'Wrist PWM: {self.wrist_pwm}')
        
        pwm_values = Int32MultiArray()
        pwm_values.data = [int(self.upper_pwm), int(self.lower_pwm), 
                          int(self.wrist_pwm[0]), int(self.wrist_pwm[1]), int(self.wrist_pwm[2])]
        self.pwm_publisher.publish(pwm_values)
        
    def control_loop(self):
        if self.start_PID == True:
            upper_current_value, lower_current_value = self.current_states
            upper_target_value, lower_target_value = self.target_states
            
            upper_control_output = self.upper_pid.update(upper_current_value, upper_target_value)
            lower_control_output = self.lower_pid.update(lower_current_value, lower_target_value)
            
            self.lower_pwm += lower_control_output
            self.upper_pwm -= upper_control_output
            
            max_pwm = 150
            self.upper_pwm = max(min(self.upper_pwm, max_pwm), -max_pwm)
            self.lower_pwm = max(min(self.lower_pwm, max_pwm), -max_pwm)
            
            if abs(upper_current_value - upper_target_value) < self.limit_margin:
                self.upper_limit_flag = True
                self.upper_pwm = 0
            else:
                self.upper_limit_flag = False
                
            if abs(lower_current_value - lower_target_value) < self.limit_margin:
                self.lower_limit_flag = True
                self.lower_pwm = 0
            else:
                self.lower_limit_flag = False
            
            if self.lower_limit_flag == True and self.upper_limit_flag == True:
                self.stop_decision = False
            
            pwm_values = Int32MultiArray()
            pwm_values.data = [int(self.upper_pwm), int(self.lower_pwm), 
                              int(self.wrist_pwm[0]), int(self.wrist_pwm[1]), int(self.wrist_pwm[2])]
            
            self.get_logger().info(
                f'Upper: {upper_current_value}->{upper_target_value} PWM:{self.upper_pwm} | '
                f'Lower: {lower_current_value}->{lower_target_value} PWM:{self.lower_pwm}'
            )
            
            if self.stop_decision:
                self.pwm_publisher.publish(pwm_values)

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
