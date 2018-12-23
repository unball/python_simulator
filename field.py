#!usr/bin/env python3
#-*- coding: utf-8 -*-
"""
    @author: Hiago dos Santos (hiagop22@gmail.com)
    @description: This module describes the Class Field and others necessary classes
    used into the Field class.
    Based on Chris Campbell's tutorial from iforce2d.net:
    'http://www.iforce2d.net/b2dtut/top-down-car'
"""

from constants import *
import sys
#from communication_ros import *
from objects_on_field.objects import (Ball, Walls, Robot, Ground )
from pygame_framework.physics_engine import *
from pygame_framework.framework import *
import math

N_ROBOTS = 3


class Field(PygameFramework):
    name = "Python simulator"
    description = "Robots controled by ros"
    #description = "If there is only one robot. Keys: accel = w, reverse = s, left = a, right = d"

    def __init__(self, *args, **kargs):
        super(Field, self).__init__()
        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)

        self.key_map = {Keys.K_w: 'up',
                        Keys.K_s: 'down',
                        Keys.K_a: 'left',
                        Keys.K_d: 'right',
                        }

        # Keep track of the pressed keys
        self.pressed_keys = set()

        self.ground = Ground(self.world)
        walls = Walls(self.world, BLUE)
        self.ball = Ball(self.world, BLUE)
        self.robots = [Robot(self.world, position=(0,10*x)) for x in range(N_ROBOTS)]

        super(Field, self).run()
        

    def Keyboard(self, key):
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.add(key_map[key])
        else:
            super(Field, self).Keyboard(key)

    def KeyboardUp(self, key):
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.remove(key_map[key])
        else:
            super(Field, self).KeyboardUp(key)

    def Step(self, settings):
        for x in range(N_ROBOTS):
            self.robots[x].update(self.pressed_keys, settings.hz)

        self.ball.update()
        self.ground.update()
        super(Field, self).Step(settings)

        
        for x in range(N_ROBOTS):
            #tractions = [self.robot.current_traction for tire in self.robots[x].tires]
            print(self.robots[x].body.position)
        #self.Print('Current tractions: %s' % tractions)

    