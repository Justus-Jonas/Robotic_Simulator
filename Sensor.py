import sympy
from sympy import solveset, atan, solve
import mpmath
import math



class Sensor():
    def __init__ (self, originialAngle, currentAngle,distance= None):
        self.angle = originialAngle
        self.distance = distance
        self.currentAngle = currentAngle


class SensorComputation():

    def __init__(self, angleRobot, map, radius):
        self.angleRobot = angleRobot
        self.map = map
        self.sensors = []
        self.radius = radius
        for x in range(0, 360, 30):
            angle = (x + angleRobot) % 360
            self.sensors.append(Sensor(x, angle))


    def find_first_contact_weaker1_right(self, x_1,x_2,y_1,y_2):
        # slope: smaller than (equals) 1 but positive
        # x_1, y_1 = position of player
        # x_2, y_2 = position of wall
        for x in range(x_1,x_2):
            y = y_1 + (y_2-y_1)/(x_2 - x_1) * (x - x_1)
            y = round(y)
            if self.map[x][y] is not None:
                return x,y
        return x_2, y_2

    def find_first_contact_stronger1_right(self, x_1,x_2,y_1,y_2):
        # slope: larger than 1 but positive
        # x_1, y_1 = position of player
        # x_2, y_2 = position of wall
        for y in range(y_1,y_2):
            x = x_1 + (x_2 - x_1)/(y_2-y_1) * (y - y_1)
            x = round(x)
            if self.map[x][y] is not None:
                return x,y
        return x_2, y_2

    def find_first_contact_stronger1_left(self, x_1,x_2,y_1,y_2):
        # slope: larger than 1 but negative (so something like -2)
        # x_1, y_1 = position of player
        # x_2, y_2 = position of wall
        for y in range(y_1,y_2, -1):
            x = x_1 + (x_2 - x_1)/(y_2-y_1) * (y - y_1)
            x = round(x)
            if self.map[x][y] is not None:
                return x,y
        return x_2, y_2

    def find_first_contact_weaker1_left(self, x_1, x_2, y_1, y_2):
        # slope: smaller than 1 but negative (so something -1 and 0)
        # x_1, y_1 = position of player
        # x_2, y_2 = position of wall
        for x in range(x_1, x_2, -1):
            y = y_1 + (y_2 - y_1) / (x_2 - x_1) * (x - x_1)
            y = round(y)
            if self.map[x][y] is not None:
                return x, y
        return x_2, y_2

    def distance(self,x_1,x_2,y_1,y_2):
        return math.sqrt((y_2-y_1)**2 + (x_2-x_1)**2)


    def calculate_distance(self, sensor,x_pos, y_pos):
    # Bresenham's line algorithm: Wikipedia: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
        x_max = self.map.shape[0]-1
        y_max = self.map.shape[1]-1

        x_edge = None
        y_edge = None

        # at y border
        intersection_x = None

        # at x border
        intersection_y = None

        find_first_contact = None

        slope = math.tan(math.radians(sensor.currentAngle))
        if 0 < sensor.currentAngle < 90:
            intersection_x = int((y_max - y_pos)/slope + x_pos)
            intersection_y = int((x_max - x_pos) * slope + y_pos)
            if slope > 1:
                find_first_contact = self.find_first_contact_stronger1_right
            else:
                find_first_contact = self.find_first_contact_weaker1_right
        elif 90 < sensor.currentAngle < 180:
            intersection_x = int((y_max - y_pos) / slope + x_pos)
            intersection_y = int((0 - x_pos) * slope + y_pos)
            if slope < -1:
                find_first_contact = self.find_first_contact_stronger1_left
            else:
                find_first_contact = self.find_first_contact_weaker1_left
        elif 180 < sensor.currentAngle < 270:
            intersection_x = int((0 - y_pos) / slope + x_pos)
            intersection_y = int((0 - x_pos) * slope + y_pos)
            if slope < 1:
                find_first_contact = self.find_first_contact_stronger1_left
            else:
                find_first_contact = self.find_first_contact_weaker1_left
        elif 270 < sensor.currentAngle < 360:
            intersection_x = int((0 - y_pos) / slope + x_pos)
            intersection_y = int((x_max - x_pos) * slope + y_pos)
            if slope < -1:
                find_first_contact = self.find_first_contact_stronger1_right
            else:
                find_first_contact = self.find_first_contact_weaker1_right
        elif sensor.currentAngle == 0:
            for x_i in range(x_pos, x_max):
                if self.map[x_i][y_pos] is not None:
                    return x_i - x_pos
            return x_max-x_pos
        elif sensor.currentAngle == 90:
            for y_i in range(y_pos, y_max):
                if self.map[x_pos][y_i] is not None:
                    return y_i - y_pos
            return y_max - y_pos
        elif sensor.currentAngle == 180:
            for x_i in range(x_pos, 0, -1):
                if self.map[x_i][y_pos] is not None:
                    return x_pos - x_i
            return x_pos
        elif sensor.currentAngle == 270:
            for y_i in range(y_pos, 0, -1):
                if self.map[x_pos][y_i] is not None:
                    return y_pos - y_i
            return y_pos

        if (0 <= intersection_x <= x_max):
            x_edge = intersection_x
            y_edge = y_max
        elif (0 <= intersection_y <= y_max):
            x_edge = x_max
            y_edge = intersection_y
        else:
            print('no intersection error')
            print(sensor.currentAngle)
            print(intersection_x)
            print(intersection_y)
        if (0 < intersection_x < x_max) and (0 < intersection_y < y_max):
            print('two intersection error')
            print(sensor.currentAngle)
            print(slope)
            print(math.radians(sensor.currentAngle))
            print(intersection_x)
            print(intersection_y)
        x, y = find_first_contact(x_pos, x_edge, y_pos, y_edge)
        return self.distance(x_pos, x, y_pos, y)



    def update_distance(self, angleRobot, map, x_pos, y_pos):
            self.angleRobot = angleRobot
            self.map = map
            distances = []

            for sensor in self.sensors:
                sensor.currentAngle = (sensor.angle + angleRobot) % 360
                sensor.distance = self.calculate_distance(sensor,x_pos,y_pos)-self.radius
                distances.append(sensor)
            return distances
