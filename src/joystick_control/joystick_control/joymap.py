#Enter states here.
#Choose to hardcode in terms of Encoder values as done here.

DROP_STATE = [315,820,0,0,0]
NINETY_DEGREE_STATE = [650,250,0,0,0]
HOME_STATE = [365,785,0,0,0]
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
        target_state =  JoyMap[tuple(joyArray)]
    return target_state