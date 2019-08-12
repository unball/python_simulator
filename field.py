#!usr/bin/env python3
#   -*- coding: utf-8 -*-
#      @author: Hiago dos Santos Rabelo (hiagop22@gmail.com)
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
        self.trajectory = Trajectory(self.world, WHITE, 0.2)
        
        # left
        # 0 : x = -63, y =0, theta = pi/2
        # 1 : x = -58, y = 0, theta = 0
        # 2 : x = -5, y = 0, theta = 0

        # right
        # 0 : x = +63, y =0, theta = -pi/2
        # 1 : x = +58, y = 0, theta = -pi
        # 2 : x = +5, y = 0, theta = -pi

        self.init_x_position = (65, 57 , 40, 30, 6)

        if field_side == 'left':
            self.robots_allies = [Robot(self.world, team_color, 'left', 
                                 position=(-self.init_x_position[x], x)
                                 ) for x in range(self.num_allies)]
            
            self.robots_opponents = [Robot(self.world, not team_color, 'right', 
                                    position=(self.init_x_position[x], x)
                                    ) for x in range(self.num_opponents)]
        else:
            self.robots_allies = [Robot(self.world, team_color, 'right', 
                                 position=(self.init_x_position[x], x)
                                 ) for x in range(self.num_allies)]
            self.robots_opponents = [Robot(self.world, not team_color, 'left', 
                                 position=(-self.init_x_position[x], x) 
                                 ) for x in range(self.num_opponents)]

        super(Field, self).run()
        
    def Keyboard(self, key):
        super(Field, self).Keyboard(key)

    def KeyboardUp(self, key):
        super(Field, self).KeyboardUp(key)

    def update_phisics(self, settings):
        if not self.pause:
            for x in range(self.num_allies):
                self.robots_allies[x].update(self.ang_and_lin_speed[x], settings.hz)

            for x in range(self.num_opponents):
                self.robots_opponents[x].update(self.ang_and_lin_speed[self.num_allies +x], 
                                                settings.hz)
        else:
            for x in range(self.num_allies):
                self.robots_allies[x].update((0,0), settings.hz)
            
            for x in range(self.num_opponents):
                    self.robots_opponents[x].update((0, 0), settings.hz)

        self.ball.update()
        self.ground.update()
    
        robots_allies = []
        for allie in range(self.num_allies):
        	self.robots_allies[allie].body.angle %= (2*math.pi)
        	angle = self.robots_allies[allie].body.angle
        	
        	if angle > math.pi:
        		angle = -(2*math.pi - angle)
        	robots_allies.append((self.robots_allies[allie].body.position, angle))

        robots_opponents = []
        for opponent in range(self.num_opponents):
        	self.robots_opponents[opponent].body.angle %= (2*math.pi)
        	angle = self.robots_opponents[opponent].body.angle

        	if angle > math.pi:
        		angle = -(2*math.pi - angle)
        	robots_opponents.append((self.robots_opponents[opponent].body.position, angle))

        RunRos.update(self, robots_allies, robots_opponents, self.ball.body.position)
        #print("angular_simulator>>>", self.robots_allies[0].body.angularVelocity)

    def Step(self, settings):
        self.update_phisics(settings)

        super(Field, self).Step(settings)

        for x in range(self.num_allies):
            self.robots_allies[x].update_colors()
        for x in range(self.num_opponents):
            self.robots_opponents[x].update_colors()
        # p = ((0.2,0.8,1.2, 1.6), (0.2,0.8,1.2, 1.6))
        self.trajectory.update(self.trajectory_x, self.trajectory_y)
