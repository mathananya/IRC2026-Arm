from arm_manual_control.arm_manual_control.pid2 import PIDController
import time

Kp = 0.5
Ki = 0
Kd = 0

# Kp = 15.0
# Ki = 10.0
# Kd = 0.02

upper_pid = PIDController(Kp, Ki, Kd, integral_max=10, integral_min=-10, margin_of_error=3)
lower_pid = PIDController(Kp, Ki, Kd, integral_max=10, integral_min=-10, margin_of_error=3)
target_state = [300, 509]
tupper = target_state[0]
tlower = target_state[1]
current_upper = 500
current_lower = 607

PWMMax = 150


stop_Pid = False

while(stop_Pid == False):
    time.sleep(0.02)
    upperPwm = upper_pid.update(current_upper, tupper)
    lowerPwm = lower_pid.update(current_lower, tlower)

    if(abs(upperPwm) > PWMMax):
        if(upperPwm > 0):
            upperPwm = PWMMax
        else:
            upperPwm = -PWMMax


    if(abs(lowerPwm) > PWMMax):
        if(lowerPwm > 0):
            lowerPwm = PWMMax
        else:
            lowerPwm = -PWMMax

    print(f"Recieved PWM : {upperPwm}, {lowerPwm}")
    print(f"current states : {current_upper}, {current_lower}")
    print(f"Target States : {tupper}, {tlower}")
    if(tupper > current_upper):
        current_upper+=10
    else:
        current_upper-=10
    if(tlower > current_lower):
        current_lower+=10
    else:
        current_lower-=10

    if(abs(current_upper - tupper) < 10 and abs(current_lower - tlower) < 10):
        stop_Pid = True
