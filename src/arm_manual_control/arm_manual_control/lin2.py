import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import matplotlib.pyplot as plt
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
        self.pwm_values = [0] * 5  # Store current PWM values
        self.pid_enabled = False  # Start loop only when a target state is received

        # Timer for the control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop

        # Initialize matplotlib for plotting
        self.fig, self.ax = plt.subplots(2, 1, figsize=(10, 8))
        self.time_points = []
        self.current_data_points = [[] for _ in range(5)]
        self.target_data_points = [[] for _ in range(5)]
        self.pwm_data_points = [[] for _ in range(5)]
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
        pwm=[0,0,0,0,0]
        for i in range(5):  # Loop through all 5 actuators
            # Calculate error
            self.errors[i] = self.target_states[i] - self.current_states[i]

            # PID calculations
            self.integrals[i] += self.errors[i]
            derivative = self.errors[i] - self.previous_errors[i]

            pwm[i] += (self.kp * self.errors[i]) + (self.ki * self.integrals[i]) + (self.kd * derivative)

            # Clamp PWM values between -150 and 150
            pwm[i] = max(-150, min(150, int(pwm[i])))

            pwm_values.append(pwm[i])
            self.pwm_values[i] = pwm[i] # Store for plotting

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
        """Update the real-time plot of target vs current encoder data and PWM values."""
        current_time = time.time() - self.start_time
        self.time_points.append(current_time)

        # Append current and target values for each actuator
        for i in range(5):
            self.current_data_points[i].append(self.current_states[i])
            self.target_data_points[i].append(self.target_states[i])
            self.pwm_data_points[i].append(self.pwm_values[i])

        # Clear and re-plot
        self.ax[0].clear()
        self.ax[1].clear()

        # Plot target vs current encoder data
        for i in range(5):
            self.ax[0].plot(self.time_points, self.current_data_points[i], label=f'Actuator {i+1} Current', linestyle='-')
            self.ax[0].plot(self.time_points, self.target_data_points[i], label=f'Actuator {i+1} Target', linestyle='--')

        self.ax[0].set_title('Target vs Current Encoder Data')
        self.ax[0].set_xlabel('Time (s)')
        self.ax[0].set_ylabel('Position')
        self.ax[0].legend()
        self.ax[0].grid(True)

        # Plot PWM values
        for i in range(5):
            self.ax[1].plot(self.time_points, self.pwm_data_points[i], label=f'Actuator {i+1} PWM', linestyle='-')

        self.ax[1].set_title('PWM Values')
        self.ax[1].set_xlabel('Time (s)')
        self.ax[1].set_ylabel('PWM (-150 to 150)')
        self.ax[1].legend()
        self.ax[1].grid(True)

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
