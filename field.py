#!usr/bin/env python3
#-*- coding: utf-8 -*-
"""
    @author: Hiago dos Santos (hiagop22@gmail.com)
    @description: This module describes the Class Field and others necessary classes
    used into the Field class.
    Based on Chris Campbell's tutorial from iforce2d.net:
    'http://www.iforce2d.net/b2dtut/top-down-car'
"""

import sys
import math
from constants import *
from communication_ros import *
from objects_on_field.objects import *
from pygame_framework.framework import *


class Field(PygameFramework, RunRos):
    name = "Python simulator"
    description = "Robots controled by ros"

    def __init__(self, num_allies, num_opponents, team_color, publish_topic):
        PygameFramework.__init__(self)
        RunRos.__init__(self, publish_topic)
        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)

        # Keep track of the pressed keys
        self.pressed_keys = set()
        
        self.num_allies = num_allies
        self.num_opponents = num_opponents

        self.ground = Ground(self.world)
        walls = Walls(self.world, BLUE)
        self.ball = Ball(self.world, BLUE)
        self.robots_allies = [Robot(self.world, position=(-10, x)) for x in range(self.num_allies)]
        self.robots_opponents = [Robot(self.world, position=(10, x)) for x in range(self.num_opponents)]

        super(Field, self).run()
        

    def Keyboard(self, key):
        super(Field, self).Keyboard(key)

    def KeyboardUp(self, key):
        super(Field, self).KeyboardUp(key)

    def Step(self, settings):
        if not self.pause:
            for x in range(self.num_allies):
                self.robots_allies[x].update(self.ang_and_lin_speed[x], settings.hz)
        else:
            for x in range(self.num_allies):
                self.robots_allies[x].update((0,0), settings.hz)

        for x in range(self.num_opponents):
                self.robots_opponents[x].update((), settings.hz)

        self.ball.update()
        self.ground.update()

        robots = [(self.robots_allies[x].body.position, 
                    self.robots_allies[x].body.angle) for x in range(self.num_allies)]
        RunRos.update(self, robots, self.ball.body.position)

        super(Field, self).Step(settings)
        #else:

        
        #for x in range(N_ROBOTS):
            #tractions = [self.robot.current_traction for tire in self.robots[x].tires]
            #print(self.robots[x].body.position)
        #self.Print('Current tractions: %s' % tractions)

