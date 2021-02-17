import pygame
from robot import Robot
from pygame.locals import *
from config import *

pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

x, y, theta = ROBOT_INITIAL_POSITION
robot = Robot(x=x,
              y=y,
              theta=theta,
              l=ROBOT_WIDTH, 
              color=ROBOT_COLOR,
              line_color=LINE_COLOR)

active = True
while active:
    screen.fill(SCREEN_FILL)
    velocities  = {'vr': 0, 'vl': 0}
    events = pygame.event.get()
    keys = pygame.key.get_pressed()

    for event in events:
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                active = False
    if keys[K_i]:
        velocities['vr'] = VR_FORWARD
    if keys[K_e]:
        velocities['vl'] = VL_FORWARD
    if keys[K_k]:
        velocities['vr'] = VR_BACKWARD
    if keys[K_d]:
        velocities['vl'] = VL_BACKWARD

    robot.move(velocities['vr'], velocities['vl'])
    robot.update_position(screen=screen)
    pygame.display.update()
