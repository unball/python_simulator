#!usr/bin/env python3
#   -*- coding: utf-8 -*-
#      @author: UnBall (equipe.unball@gmail.com)
# @description: This module describes the Class Field that has all necessary objects 
#               on field.
#               Based on Chris Campbell's tutorial from iforce2d.net:
#               'http://www.iforce2d.net/b2dtut/top-down-car'


import sys
import math
from constants import *
from communication_ros import *
from objects_on_field.objects import *
from pygame_framework.framework import *


class Field(PygameFramework, RunRos):
    name = "Python simulator"
    description = "Robots controled by ros"

    def __init__(self, num_allies, num_opponents, team_color, field_side, publish_topic):
        PygameFramework.__init__(self)
        RunRos.__init__(self, publish_topic)
        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)

        # Objects on field 
        self.num_allies = num_allies
        self.num_opponents = num_opponents

        self.ground = Ground(self.world)
        walls = Walls(self.world, BLUE)
        self.ball = Ball(self.world, BLUE)

        if field_side == 'left':
            self.robots_allies = [Robot(self.world, position=(-10, x), angle=0
                                 ) for x in range(self.num_allies)]
            self.robots_opponents = [Robot(self.world, position=(10, x), angle=math.pi
                                 ) for x in range(self.num_opponents)]
        else:
            self.robots_allies = [Robot(self.world, position=(10, x), angle=math.pi
                                 ) for x in range(self.num_allies)]
            self.robots_opponents = [Robot(self.world, position=(-10, x), angle=0
                                 ) for x in range(self.num_opponents)]

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

        robots_allies = [(self.robots_allies[x].body.position, 
                    self.robots_allies[x].body.angle) for x in range(self.num_allies)]
        robots_opponents = [(self.robots_opponents[x].body.position, 
                    self.robots_opponents[x].body.angle) for x in range(self.num_opponents)]

        RunRos.update(self, robots_allies, robots_opponents, self.ball.body.position)

        super(Field, self).Step(settings)