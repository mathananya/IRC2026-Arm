#Enter states here.
#Choose to hardcode in terms of Encoder values as done here.

DROP_STATE = [315,820,0,0,0]
NINETY_DEGREE_STATE = [627, 167, 0, 0, 0]
HOME_STATE = [705, 534, 0, 0, 0]
PICK_STATE = [590,380,0,0,0]
ARM_EXTEND = [850,410,0,0,0]

def mapJoystickToAction(joyArray):
    target_state = None
    JoyMap = {
        (0,0,1,0,0) : DROP_STATE,
        (0,1,0,0,0) : NINETY_DEGREE_STATE,
        (1,0,0,0,0) : HOME_STATE,
        (0,0,0,1,0) : PICK_STATE,
        (0,0,0,0,1) : ARM_EXTEND
    }
    if tuple(joyArray) in JoyMap:
        target_state =  JoyMap.get(tuple(joyArray))
    return target_state


def mapJoyAxes(joyAxes):
    pwm_val = [0,0,0,0,0]
    VAL = 100
    if(joyAxes[1] == 1):
        pwm_val[1] -= VAL 
    elif(joyAxes[1] == -1):
        pwm_val[1] += VAL
    if(joyAxes[5] == 1):
        pwm_val[0] += VAL
    elif(joyAxes[5] == -1):
        pwm_val[0] -= VAL
    #gripper yet to add, ask what key on keyboard was doing gripper and 
    # the decided switch is joyAxes[3]
    return pwm_val