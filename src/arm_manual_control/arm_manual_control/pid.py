import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, integral_max=100, integral_min=-100, margin_of_error=50):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.integral_max = integral_max
        self.integral_min = integral_min
        self.last_timestamp = time.time()
        self.margin_of_error = margin_of_error

    def update(self, current_value, setpoint):
        self.current_timestamp = time.time()
        self.dt = self.current_timestamp - self.last_timestamp

        self.error = setpoint - current_value
        print(self.error)
        # Check if the error is within the margin
        if abs(self.error) <= self.margin_of_error:
            self.output = 0
        else:
            self.integral += self.error * self.dt
            self.integral = max(min(self.integral, self.integral_max), self.integral_min)  # Anti-windup
            self.derivative = (self.error - self.prev_error) / self.dt

            self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
            print(self.output)
        self.prev_error = self.error
        self.last_timestamp = self.current_timestamp

        return self.output
