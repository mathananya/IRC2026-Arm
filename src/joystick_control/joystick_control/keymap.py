def keytoState(key):
    VAL = 50

    darray = [0, 0, 0, 0, 0]
    m = {
    'w': (0, 1),
    's': (0, -1),
    'a': (1, 1),
    'd': (1, -1),
    'r': (2, 1),
    'f': (2, -1),
    't': (3, 1),
    'g': (3, -1),
    'y': (4, 1),
    'h': (4, -1),
    }
    
    if key in m :
        pos = m[key][0]
        factor = m[key][1]

        darray[pos] += VAL * factor

    return darray