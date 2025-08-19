import math

def distance(point1, point2):
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]

    return math.sqrt(dx**2 + dy**2)

#get yaw from two 2d points (pointing from point1 to point2)
def getyaw(point1, point2):
    a = point2[1] - point1[1]
    b = point2[0] - point1[0]
    yaw = -1*math.atan2(b, a)  + math.pi/2.0
    return yaw

#rotate vector by angle
def rotate(x: float, y: float, alpha:float) -> tuple[float, float]:
    xrot = x*math.cos(alpha) - y*math.sin(alpha)
    yrot = x*math.sin(alpha) + y*math.cos(alpha)
    return xrot, yrot

#h is hue in range 0 to 360
#returns bgr in rage 0 to 255
def hueToBGR(h: float):
    h = h/360

    kr = 5+h*6 % 6
    kg = 3+h*6 % 6
    kb = 1+h*6 % 6

    r = 1 - max(min(kr, 4-kr, 1), 0)
    g = 1 - max(min(kg, 4-kg, 1), 0)
    b = 1 - max(min(kb, 4-kb, 1), 0)

    return b*255, g*255, r*255