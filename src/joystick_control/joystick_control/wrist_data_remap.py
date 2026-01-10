map = {
    (0.0, 1.0) : (-100, -100),
    (0.0, -1.0) : (100, 100),
    (1.0, 0.0) : (100, -100),
    (-1.0, 0.0) : (-100, 100),
    (0.0, 0.0) : (0.0, 0.0)
    } 

def wrist_map(data):
    if tuple(data) in map:
        wrist_pwm = map.get(tuple(data))
    return wrist_pwm