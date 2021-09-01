#!/usr/bin/env python

import pygame

pygame.init()

num_joysticks = pygame.joystick.get_count()
if num_joysticks == 0:
    raise ValueError("No joysticks attached!")

joystick = pygame.joystick.Joystick(0)
joystick.init()

while True:
    for event in pygame.event.get():
        pass

    [throttle, rudder, elevator] = int(joystick.get_axis(1)*100), int(joystick.get_axis(3)*100), int(joystick.get_axis(2)*100)
    print("{},{},{}".format(throttle,rudder,elevator))