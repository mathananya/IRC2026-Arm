#Enter states here.
#Choose to hardcode in terms of Encoder values as done here.

# DROP_STATE = [820, 315,0,0,0]
# HOME_STATE = [415,885,0, 0, 0]
# NINETY_DEGREE_STATE = [644,153,0,0,0]
# VIEW_STATE = [440,580,0,0,0]
# PICK_STATE = [590,380,0,0,0]
# ARM_EXTEND = [850,410,0,0,0]

# DROP_STATE = [820, 315,0,0,0]
PICK_STATE = [570,340,0,0,0]
DROP_STATE = [280,570,0,0,0]
HOME_STATE = [270,523,0,0,0]
NINETY_DEGREE_STATE = [419,55,0,0,0]
VIEW_STATE = [250,490,0,0,0]
# ARM_EXTEND = [850,410,0,0,0]

def mapJoystickToAction(joyArray):
    target_state = None
    JoyMap = {
        (0,0,1,0,0) : DROP_STATE,
        (0,1,0,0,0) : NINETY_DEGREE_STATE,
        (1,0,0,0,0) : HOME_STATE,
        (0,0,0,1,0) : VIEW_STATE,
        (0,0,0,0,1) : PICK_STATE
    }
    if tuple(joyArray) in JoyMap:
        target_state =  JoyMap.get(tuple(joyArray))
    return target_state


def mapJoyAxes(joyAxes):
    pwm_val = [0,0,0,0,0]
    VAL = 200
    if(joyAxes[1] == 1):
        pwm_val[0] -= VAL 
    elif(joyAxes[1] == -1):
        pwm_val[0] += VAL
    if(joyAxes[5] == 1):
        pwm_val[1] += VAL
    elif(joyAxes[5] == -1):
        pwm_val[1] -= VAL
    return pwm_val
