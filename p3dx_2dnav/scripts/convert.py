import json
from math import sin, cos, radians, pi

class Convert():
    def robot_to_human(self, x0, y0, theta, r=0.7):
        point = []
        theta = (theta + 180) * pi / 180
        x = x0 + r * cos(theta)
        y = y0 + r * sin(theta)
        point.append(x)
        point.append(y)
        return point

    def human_to_robot(self, x0, y0, theta, r=0.7):
        point = []
        theta = theta * pi / 180
        x = x0 + r * cos(theta)
        y = y0 + r * sin(theta)
        point.append(x)
        point.append(y)
        return point
