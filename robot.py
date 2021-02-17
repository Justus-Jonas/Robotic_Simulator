import pygame
import math
import numpy as np

class Robot:

    def __init__(self, x, y, theta, l, color, line_color):
        self.x = x
        self.y = y
        self.theta = theta
        self.l = l
        self.r = l/2
        self.R = None
        self.w = None
        self.color = color
        self.line_color = line_color

    def update_position(self, screen):
        adjusted_y = screen.get_height() - self.y
        self.body = pygame.draw.circle(screen,
                                       self.color, 
                                       (self.x, self.y),
                                       self.r)
        self.line = pygame.draw.line(screen, 
                                     self.line_color, 
                                     (self.x, self.y), 
                                     (self.x + self.r * math.cos(-self.theta), self.y -  self.r * math.sin(-self.theta))
                                    )
    def calculate_rw(self, vr, vl):
        self.R = (self.l/2)*((vr + vl) / (vr - vl))
        self.w = (vr - vl) / self.l

    def calculate_ICC(self):
        return (self.x - self.R * math.sin(self.theta), self.y + self.R * math.cos(self.theta))

    def move(self, vr, vl):
        # check collisions
            ###

        if vr == vl:
            self.x -= math.cos(-self.theta) * vr/2
            self.y -= math.sin(self.theta) * vr/2

            return 
        
        self.calculate_rw(vr, vl)
        icc = self.calculate_ICC()
        self.rotation_matrix = np.array([[math.cos(self.w),  math.sin(self.w), 0], 
                                         [-math.sin(self.w),  math.cos(self.w), 0], 
                                         [0, 0, 1]])
        self.distance_matrix = np.array([self.x - icc[0], self.y - icc[1], self.theta]).reshape(-1, 1)
        self.increment_matrix = np.array([icc[0], icc[1], self.w]).reshape(-1, 1)

        new_position =  np.dot(self.rotation_matrix, self.distance_matrix) + self.increment_matrix
        self.x = new_position[0][0]
        self.y = new_position[1][0]
        self.theta = new_position[2][0]
