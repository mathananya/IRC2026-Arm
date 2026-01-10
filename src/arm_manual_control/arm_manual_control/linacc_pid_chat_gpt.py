import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
import numpy as np
import time

class LinearActuatorPID(Node):
    def __init__(self):
        super().__init__('linear_actuator_pid')

        # Subscriptions
        self.current_state_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_encoder_data', self.current_state_callback, 10)
        self.target_state_subscriber = self.create_subscription(
            Int32MultiArray, 'arm_target_states', self.target_state_callback, 10)

        # Publisher
        self.pwm_publisher = self.create_publisher(Int32MultiArray, 'arm_pwm_commands', 10)

        # PID parameters
        self.kp = 1.0
        self.ki = 0.01
        self.kd = 0.1

        # Data storage
        self.current_states = [0] * 5  # Current positions of the actuators
        self.target_states = [0] * 5  # Target positions of the actuators
        self.errors = [0] * 5
        self.integrals = [0] * 5
        self.previous_errors = [0] * 5
        self.pid_enabled = False  # Start loop only when a target state is received

        # Timer for the control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop

        # Initialize matplotlib for plotting
        self.fig, self.ax = plt.subplots()
        self.time_points = []
        self.current_data_points = [[] for _ in range(5)]
        self.target_data_points = [[] for _ in range(5)]
        self.start_time = time.time()

        plt.ion()  # Enable interactive mode
        self.fig.show()
        self.fig.canvas.draw()

    def current_state_callback(self, msg):
        """Callback to update the current encoder data."""
        self.current_states = msg.data

    def target_state_callback(self, msg):
        """Callback to update the target states and enable the PID loop."""
        self.target_states = msg.data
        self.pid_enabled = True

    def control_loop(self):
        """PID control loop to compute PWM values."""
        if not self.pid_enabled:
            return

        pwm_values = []
        for i in range(5):  # Loop through all 5 actuators
            # Calculate error
            self.errors[i] = self.target_states[i] - self.current_states[i]

            # PID calculations
            self.integrals[i] += self.errors[i]
            derivative = self.errors[i] - self.previous_errors[i]

            pwm = (self.kp * self.errors[i]) + (self.ki * self.integrals[i]) + (self.kd * derivative)

            # Clamp PWM values between 0 and 150
            pwm = max(0, min(150, int(pwm)))

            pwm_values.append(pwm)

            # Update previous error
            self.previous_errors[i] = self.errors[i]

        # Publish PWM commands
        pwm_msg = Int32MultiArray()
        pwm_msg.data = [pwm_values[0],pwm_values[1],0,0,0]
        self.pwm_publisher.publish(pwm_msg)

        # Log PWM commands
        self.get_logger().info(f"PWM Commands: {pwm_values}")

        # Update the plot
        self.update_plot()

    def update_plot(self):
        """Update the real-time plot of target vs current encoder data."""
        current_time = time.time() - self.start_time
        self.time_points.append(current_time)

        # Append current and target values for each actuator
        for i in range(5):
            self.current_data_points[i].append(self.current_states[i])
            self.target_data_points[i].append(self.target_states[i])

        # Clear and re-plot
        self.ax.clear()
        for i in range(5):
            self.ax.plot(self.time_points, self.current_data_points[i], label=f'Actuator {i+1} Current', linestyle='-')
            self.ax.plot(self.time_points, self.target_data_points[i], label=f'Actuator {i+1} Target', linestyle='--')
            
        self.ax.set_title('Target vs Current Encoder Data')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Position')
        self.ax.legend()
        self.ax.grid(True)

        # Redraw the figure
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = LinearActuatorPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')  # Close the plot on shutdown

if __name__ == '__main__':
    main()
