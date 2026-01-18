import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, integral_max=100, integral_min=-100, margin_of_error=50, flag=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.integral_max = integral_max
        self.integral_min = integral_min
        self.last_timestamp = time.time()
        self.margin_of_error = margin_of_error
        self.flag = flag
        self.output = 0
    
    def update(self, current_value, setpoint):
        self.current_timestamp = time.time()
        self.dt = self.current_timestamp - self.last_timestamp
        
        
        # if self.dt < 0.001:
        #     return self.output
        if self.dt > 1.0: 
            self.dt = 0.02
        
        self.error = setpoint - current_value
        
        # print(f"Error: {self.error}, dt: {self.dt:.4f}")
        
        # Check if the error is within the margin
        # if abs(self.error) <= self.margin_of_error:
        #     self.output = 0
        
        
        self.integral += self.error * self.dt
            # self.integral = max(min(self.integral, self.integral_max), self.integral_min)  # Anti-windup
        self.integral = min(self.integral, self.integral_max)
            
        self.derivative = (self.error - self.prev_error) / self.dt
            
        self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        self.prev_error = self.error
        
        # print(f"Output: {self.output:.2f}")
        if(self.flag == 0):
            print("Upper PID VALS : ")
            print(f"P: {self.Kp * self.error:.2f}, I: {self.Ki * self.integral:.2f}, D: {self.Kd * self.derivative:.2f}")
        else:
            print("LOWER PID VALS : ")
            print(f"P: {self.Kp * self.error:.2f}, I: {self.Ki * self.integral:.2f}, D: {self.Kd * self.derivative:.2f}")
            
            # self.prev_error = self.error
        
        # print(f"Output: {self.output:.2f}")
        
        self.last_timestamp = self.current_timestamp

        # return self.output
        if(self.flag==1):
            return -self.output
        else:
            return self.output
    
    def updateError(self, errorVal):
        self.prev_error = errorVal

    def updateIntegral(self, integVal):
        self.integral = integVal
