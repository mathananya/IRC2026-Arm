import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
import time

from pid import PIDController

class LinAccController(Node):
    def __init__(self):
        super().__init__('linacc_controller')

        # Initialize PID controllers
        Kp = 10
        Ki = 0.02
        Kd = 0.1
        self.upper_pid = PIDController(Kp, Ki, Kd, integral_max=100, integral_min=-100, margin_of_error=2.5)
        self.lower_pid = PIDController(Kp, Ki, Kd, integral_max=100, integral_min=-100, margin_of_error=2.5)

        # ROS 2 Subscribers
        self.current_state_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_encoder_data', self.current_state_callback, 10)
        self.target_state_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_target_states', self.target_state_callback, 10)

        # ROS 2 Publisher
        self.pwm_publisher = self.create_publisher(Int32MultiArray, 'arm_pwm_commands', 10)

        # Timer
        self.timer = self.create_timer(0.01, self.control_loop)

        # State variables
        self.current_states = [0.0, 0.0]  # [lower_current, upper_current]
        self.target_states = [0.0, 0.0]   # [lower_target, upper_target]
	
	self.start_moving=False
    def current_state_callback(self, msg):
        # Extract lower and upper actuator states from the encoder data
        self.current_states = msg.data[:2]

    def target_state_callback(self, msg):
        self.target_states = msg.data[:2]
        self.start_moving=True

    def control_loop(self):
    	if(self.start_moving==True):
        lower_current_value, upper_current_value = self.current_states
        lower_target_value, upper_target_value = self.target_states

        # PID Update
        upper_control_output = self.upper_pid.update(upper_current_value, upper_target_value)
        lower_control_output = self.lower_pid.update(lower_current_value, lower_target_value)

        # Limit control outputs to max PWM of 95
        max_pwm = 150
        upper_control_output = max(min(upper_control_output, max_pwm), -max_pwm)
        lower_control_output = max(min(lower_control_output, max_pwm), -max_pwm)

        # Publish PWM values
        pwm_values = Int32MultiArray()
        pwm_values.data = [int(lower_control_output), int(upper_control_output),0,0,0]
        self.pwm_publisher.publish(pwm_values)

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
