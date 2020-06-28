#!usr/bin/env python3
#   -*- coding: utf-8 -*-
#      @author: Hiago dos Santos Rabelo (hiagop22@gmail.com)
#      @Project's Property: UnBall (equipe.unball@gmail.com)
# @description: This module describes the Class Field that has all necessary objects 
#               on field.
#               Based on Chris Campbell's tutorial from iforce2d.net:
#               'http://www.iforce2d.net/b2dtut/top-down-car'



import sys
import math
import numpy as np
from constants import *
from objects_on_field.objects import *
from pygame_framework.framework import *

import os

class Field(PygameFramework):
    """
    num_allies = from 0 to 5
    num_opponents = from 0 to 5
    team_color = "blue" or "yellow"
    allied_field_side = "left" or "right"
    render = False to run simulation without graphics
    cloud = True if you'are running on a online kernel

    >>SIMULATION (Options into Render mode):
    Press SPACE to Pause/Play into Simulation 
    Press ESC to return to close the simulation
    Click on a object and keep pressed to move it

    IMPORTANT:
    The simulator work with cm instead meter, so it's necessary convert the 
    number multiplying by the constants such as implemented bellow in the code
    """

    name = "Python simulator"

    def __init__(self, num_allies=3, num_opponents=3, team_color='blue', allied_field_side='left', 
                render=False, cloud=False):

        # Objects on field 
        self.num_allies = num_allies
        self.num_opponents = num_opponents

        self.render = render
        self.cloud = cloud  # Tell us if we're running our code in a kernel cloud

        if team_color == 'blue': 
            self.team_color = 0
        elif team_color == 'yellow':
            self.team_color = 1
        
        self.allied_field_side = allied_field_side

        if self.allied_field_side == 'left':
            self.x_goal_allied   = -0.75
            self.x_goal_opponent = 0.75
        else:
            self.x_goal_allied   = 0.75
            self.x_goal_opponent = -0.75

        # It will be defined latter, but now we'll create them
        self.robots_opponents = ''
        self.robots_allies    = ''

        self.action_size = ''
        self.state_size = ''

        if self.render:
            # Set the icon for the application
            directory = sys.path[0]
            if self.cloud:
                logo = pygame.image.load(directory + 'images/UnBall.png') 
            else:
                logo = pygame.image.load(directory + '/images/UnBall.png') 
            pygame.display.set_icon(logo)

    def reset(self):
        """
        Create the objects that will be simulated and return a list containing the position and angle
        of them
        """
        PygameFramework.__init__(self, self.render)
        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)

        self.lin_and_ang_speed = [(0,0) for _ in range(self.num_allies + self.num_opponents)]

        self.ground = Ground(self.world)
        walls = Walls(self.world, BLUE)
        self.ball = Ball(self.world, BLUE)
        
        # ------------------ Initial Coordinates ------------------
        # left
        # 0 : x = -65, y =0, theta = pi/2
        # 1 : x = -57, y = 0, theta = 0
        # 2 : x = -40, y = 0, theta = 0
        # 3 : x = -30, y = 0, theta = 0
        # 4 : x = -6,  y = 0, theta = 0

        # right
        # 0 : x = +65, y =0, theta = -pi/2
        # 1 : x = +57, y = 0, theta = -pi
        # 2 : x = +40, y = 0, theta = -pi
        # 3 : x = +30, y = 0, theta = -pi
        # 4 : x = +6,  y = 0, theta = -pi
        # ---------------------------------------------------------

        self.init_x_position = (65, 57 , 40, 30, 6)

        if self.allied_field_side == 'left':
            self.robots_allies = [Robot(self.world, self.team_color, 'left', 
                                 start_position=(-self.init_x_position[x], x)
                                 ) for x in range(self.num_allies)]
            
            self.robots_opponents = [Robot(self.world, not self.team_color, 'right', 
                                    start_position=(self.init_x_position[x], x)
                                    ) for x in range(self.num_opponents)]
        else:
            self.robots_allies = [Robot(self.world, self.team_color, 'right', 
                                 start_position=(self.init_x_position[x], x)
                                 ) for x in range(self.num_allies)]
            self.robots_opponents = [Robot(self.world, not self.team_color, 'left', 
                                 start_position=(-self.init_x_position[x], x) 
                                 ) for x in range(self.num_opponents)]        

        return self.next_step()

    def next_step(self):
        '''
        It'll return the positions of all elements inside the field
        '''
        # example: [[xrobot1,     xrobot2,      ....., xballPosition]
        #           [yrobot1,     yrobot2,      ....., yballPosition]
        #           [angleRobot1, angleRobot2,  .....,      0       ]]
        return_list = []
        
        # Inserting x position into the list
        pos_allies = [self.robots_allies[i].body.position[0]*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_allies)]
        pos_opponents = [self.robots_opponents[i].body.position[0]*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_opponents)]

        return_list.append(pos_allies + pos_opponents + [self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER])

        # Inserting y position into the list
        pos_allies = [self.robots_allies[i].body.position[1]*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_allies)]
        pos_opponents = [self.robots_opponents[i].body.position[1]*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_opponents)]

        return_list.append(pos_allies + pos_opponents + [self.ball.body.position[1]*CORRECTION_FACTOR_CM_TO_METER])

        # Inserting thetha angle into the list
        pos_allies = [self.robots_allies[i].body.angle for i in range(self.num_allies)]
        pos_opponents = [self.robots_opponents[i].body.angle for i in range(self.num_opponents)]

        return_list.append(pos_allies + pos_opponents + [0])

        return_array = np.array(return_list)
        
        # Numpy creates an array with zero dimension, vector wich is seen as a scalar. So,
        # to create an array wich 1 row of dimension use the bellow command
        # To see more detais look the shape of array before and after the command bellow
        # return_array = np.expand_dims(return_array, axis=0)

        return return_array

    def close(self):
        super(Field, self).close()

    def step(self, action):
        """
        Actions are w and v velocities of the allies robots
        """
        for robot in range(self.num_allies):
            self.lin_and_ang_speed[robot] = (action[robot][0]*CORRECTION_FATOR_METER_TO_CM, action[robot][1])

        super(Field, self).run()

        return (self.next_step(), self.reward(), self.done())
    
    def reward(self):
        """
        It's a simplified reward, that give +1 if the agent do a goal and
        return -1 reward if the agent receive a goal
        """
        reward = 0

        if self.allied_field_side == 'left':
            if (self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER > self.x_goal_opponent):
                reward = 1
            elif (self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER < self.x_goal_allied):
                reward = -1 
        else:
            if (self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER < self.x_goal_opponent):
                reward = 1
            elif (self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER > self.x_goal_allied):
                reward = -1 
        
        return reward

    def done(self):
        """
        Return True if ball center of mass enter inside the goal
        """
        return True if abs(self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER) > abs(self.x_goal_allied) else False
        
    def Keyboard(self, key):
        super(Field, self).Keyboard(key)

    def KeyboardUp(self, key):
        super(Field, self).KeyboardUp(key)

    def update_phisics(self, settings):
        
        if not self.pause:
            for x in range(self.num_allies):
                self.robots_allies[x].update(self.lin_and_ang_speed[x], settings.hz)

            for x in range(self.num_opponents):
                self.robots_opponents[x].update(self.lin_and_ang_speed[self.num_allies +x], 
                                                settings.hz)
        else:
            for x in range(self.num_allies):
                self.robots_allies[x].update((0,0), settings.hz)
            
            for x in range(self.num_opponents):
                    self.robots_opponents[x].update((0, 0), settings.hz)

        self.ball.update()

        if self.render:
            self.ground.update()
    
        # robots_allies = []
        for allie in range(self.num_allies):
        	self.robots_allies[allie].body.angle %= (2*math.pi)
        	# angle = self.robots_allies[allie].body.angle
        	
        	# if angle > math.pi:
        	# 	angle = -(2*math.pi - angle)
        	# robots_allies.append((self.robots_allies[allie].body.position, angle))

        # robots_opponents = []
        for opponent in range(self.num_opponents):
        	self.robots_opponents[opponent].body.angle %= (2*math.pi)
        	# angle = self.robots_opponents[opponent].body.angle

        	# if angle > math.pi:
        	# 	angle = -(2*math.pi - angle)
        	# robots_opponents.append((self.robots_opponents[opponent].body.position, angle))

    def Step(self, settings):
        self.update_phisics(settings)

        super(Field, self).Step(settings)

        if self.render:
            for x in range(self.num_allies):
                self.robots_allies[x].update_colors()
            for x in range(self.num_opponents):
                self.robots_opponents[x].update_colors()
