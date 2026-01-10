import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from pid import PIDController
import time 

class LinAccController(Node):
    def __init__(self):
        super().__init__('linacc_controller')

        # Initialize PID controllers
        Kp = 0.1
        Ki = 0.0
        Kd = 0.0 
        self.upper_pid = PIDController(Kp, Ki, Kd, integral_max=10, integral_min=-10, margin_of_error=2.5)
        self.lower_pid = PIDController(Kp, Ki, Kd, integral_max=10, integral_min=-10, margin_of_error=2.5)

        # ROS 2 Subscribers
        self.current_state_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_encoder_data', self.current_state_callback, 10)
        self.target_state_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_target_states', self.target_state_callback, 10)

        # ROS 2 Publisher
        self.pwm_publisher = self.create_publisher(Int32MultiArray, 'arm_pwm_commands', 10)

        # Timer
        self.timer = self.create_timer(0.3, self.control_loop)

        # State variables
        self.current_states = [0.0, 0.0]  # [lower_current, upper_current]
        self.target_states = [0.0, 0.0]   # [lower_target, upper_target]
        self.start_time = time.time()
        self.start_PID = False
        self.finished = False 
        self.stop_flag_upper = False
        self.stop_flag_lower = False
        self.upper_pwm =0
        self.lower_pwm =0
        self.time_points = []
    def current_state_callback(self, msg):
        # Extract lower and upper actuator states from the encoder data
        self.current_states = msg.data[:2]

    def target_state_callback(self, msg):
        self.target_states = msg.data[:2]
        print(self.target_states)
        self.start_PID = True
    def control_loop(self):
        if self.start_PID==True:
            lower_current_value, upper_current_value = self.current_states
            lower_target_value, upper_target_value = self.target_states

            # PID Update
            upper_control_output = self.upper_pid.update(upper_current_value, upper_target_value)
            lower_control_output = self.lower_pid.update(lower_current_value, lower_target_value)

            # Limit control outputs to max PWM of 95
            max_pwm = 100
            self.lower_pwm+=lower_control_output
            self.upper_pwm+=upper_control_output
            self.upper_pwm = max(min(self.upper_pwm, max_pwm), -max_pwm)
            self.lower_pwm = max(min(self.lower_pwm, max_pwm), -max_pwm)
            # self.lower_pwm+=lower_control_output
            # self.upper_pwm+=upper_control_output
            # if((upper_current_value-upper_target_value)>=25 ):
            #     self.stop_flag_upper =True
            # if((lower_current_value-lower_target_value)>=25 ):
            #     self.stop_flag_lower =True
            # if(self.stop_flag_upper==True):
            #     self.upper_pwm = 0
            # if(self.stop_flag_lower==True):
            #     self.lower_pwm = 0
            # if(self.stop_flag_lower==0 and self.stop_flag_upper ==0):
            #     self.start_PID == False
            # Publish PWM values
            pwm_values = Int32MultiArray()
            pwm_values.data = [int(self.upper_pwm),int(self.lower_pwm) ,0,0,0]
            print(upper_current_value,upper_target_value,lower_current_value,lower_target_value,self.upper_pwm,self.lower_pwm)
            self.pwm_publisher.publish(pwm_values)


    # def update_plot(self):
    #     current_time = time.time() - self.start_time
    #     self.time_points.append(current_time)


    def shutdown(self):
        self.get_logger().info("Shutting down node...")


def main(args=None):
    rclpy.init(args=args)
    node = LinAccController()
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
