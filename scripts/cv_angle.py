#!/usr/bin/env python
def calcAngle(x1, y1, x2, y2, w, h):
    """ returns the temperature in degrees Fahrenheit """
    #no, it doesn't
    originX = float(w/2)
    originY = float(h/2)

    averageX = float(float(x1) - originX + float(x2) - originX)/2
    averageY = float(originY - float(y1) + originY - float(y2))/2

    if averageX == 0:
        if averageY >= 0:
            angle = float(90)
        else:
            angle = float(270)
    else:
        angle = math.degrees(math.atan(averageY/averageX))

    # test = "angle is: " + str(angle)
    # print test

    if x1>originX:
        if y1>originY:
            angle = 360 + angle
        else:
            angle = angle
    elif x1<originX:
        if y1>originY:
            angle = 180 + angle
        else:
            angle = 180 + angle

    return angle
