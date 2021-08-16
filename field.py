#!usr/bin/env python3
#   -*- coding: utf-8 -*-
#      @author: Hiago dos Santos Rabelo (hiagop22@gmail.com)
#      @Project's Property: UnBall (equipe.unball@gmail.com)
# @description: This module describes the Class Field that has all necessary objects 
#               on field.
#               Based on Chris Campbell's tutorial from iforce2d.net:
#               'http://www.iforce2d.net/b2dtut/top-down-car'



from numpy.lib.function_base import place
from kdtree import KDTree
import sys
import math
import random
import numpy as np
from constants import *
from objects_on_field.objects import *
from pygame_framework.framework import *
import time

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

        self.max_v = 200 # cm/s
        self.max_w = 4*2*math.pi

        self.action_size = self.num_allies*2 # num_allies * v,w
        self.state_size = ''

        self.previous_ball_potential = None

        if self.render:
            # Set the icon for the application
            directory = sys.path[0]
            if self.cloud:
                logo = pygame.image.load(directory + 'images/UnBall.png') 
            else:
                logo = pygame.image.load(directory + '/images/UnBall.png') 
            pygame.display.set_icon(logo)

    def reset_larc_rules(self):
        """
        Create the objects that will be simulated and return a list containing the position and angle
        of them
        """
        PygameFramework.__init__(self, self.render)
        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)

        self.lin_and_ang_speed = [(0,0) for _ in range(self.num_allies + self.num_opponents)]

        self.ground = Ground(self.world)
        self.walls = Walls(self.world, BLUE)
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

    def reset_random_init_pos(self):
        """
        Create the objects that will be simulated and return a list containing the position and angle
        of them
        """
        PygameFramework.__init__(self, self.render)
        # Top-down -- no gravity in the screen plane
        def x(): return random.uniform(-self.ground.length/2 + 10,
                                       self.ground.length/2 - 10)

        def y(): return random.uniform(-self.ground.width/2 + 10,
                                       self.ground.width/2 - 10)

        def theta(): return random.uniform(0, 2*math.pi)

        self.world.gravity = (0, 0)
        self.step_episode = 0

        self.init_time_episode = time.time()

        self.lin_and_ang_speed = [(0,0) for _ in range(self.num_allies + self.num_opponents)]

        self.ground = Ground(self.world)
        self.walls = Walls(self.world, BLUE)
        ball_pos = [x(), y()]
        self.ball = Ball(self.world, BLUE, position=ball_pos)

        init_pos = [ball_pos]

        min_dist = 10
        places = KDTree()
        places.insert(ball_pos)

        for i in range(self.num_allies):
            pos = [x(), y()]
            while places.get_nearest(pos)[1] < min_dist:
                pos = [x(), y()]

            places.insert(pos)
            init_pos.append(pos)
        
        for i in range(self.num_opponents):
            pos = [x(), y()]
            while places.get_nearest(pos)[1] < min_dist:
                pos = [x(), y()]

            places.insert(pos)
            init_pos.append(pos)

        self.robots_allies = [Robot(self.world, self.team_color, num_robot=x,
                                start_position=init_pos[1 + x], angle=theta(), 
                                y_predefined=False) for x in range(self.num_allies)]
        
        self.robots_opponents = [Robot(self.world, not self.team_color, num_robot=x, 
                                start_position=init_pos[1 + self.num_allies + x], angle=theta(),y_predefined=False,
                                ) for x in range(self.num_opponents)]

        return self.next_step()


    def next_step(self):
        '''
        It'll return the positions of all elements inside the field
        '''
        return_dict = {'allie': {}, 'opponent': {}, 'ball': {}}
        
        # X, y positions
        pos_x_allies = [self.robots_allies[i].body.position[0]*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_allies)]
        pos_y_allies = [self.robots_allies[i].body.position[1]*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_allies)]
        
        pos_x_opponents = [self.robots_opponents[i].body.position[0]*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_opponents)]
        pos_y_opponents = [self.robots_opponents[i].body.position[1]*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_opponents)]

        # Angular and linear velocities
        v_allies = [self.robots_allies[i].body.linearVelocity*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_allies)]
        w_allies = [self.robots_allies[i].body.angularVelocity for i in range(self.num_allies)]

        v_opponents = [self.robots_opponents[i].body.linearVelocity*CORRECTION_FACTOR_CM_TO_METER for i in range(self.num_opponents)]
        w_opponents = [self.robots_opponents[i].body.angularVelocity for i in range(self.num_opponents)]   

        # Theta angles
        theta_allies = [self.robots_allies[i].body.angle for i in range(self.num_allies)]
        theta_opponents = [self.robots_opponents[i].body.angle for i in range(self.num_opponents)]

        # return_array = np.array(return_list)
        
        # Numpy creates an array with zero dimension, vector wich is seen as a scalar. So,
        # to create an array wich 1 row of dimension use the bellow command
        # To see more detais look the shape of array before and after the command bellow
        # return_array = np.expand_dims(return_array, axis=0)

        for i in range(self.num_allies):
            return_dict['allie'][i] = {'pos_xy': np.array([pos_x_allies[i], pos_y_allies[i]]), 
                                        'theta': theta_allies[i], 
                                        'v': np.array([v_allies[i][0],v_allies[i][1]]),
                                        'w': w_allies[i]}

        for i in range(self.num_opponents):
            return_dict['opponent'][i] = {'pos_xy': np.array([pos_x_opponents[i], pos_y_opponents[i]]), 
                                        'theta': theta_opponents[i], 
                                        'v': np.array([v_opponents[i][0],v_opponents[i][1]]),
                                        'w': w_opponents[i]}
        
        return_dict['ball'] = {'pos_xy':np.array([self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER, 
                                        self.ball.body.position[1]*CORRECTION_FACTOR_CM_TO_METER]), 
                                'v': np.array([self.ball.body.linearVelocity[0]*CORRECTION_FACTOR_CM_TO_METER,
                                            self.ball.body.linearVelocity[1]*CORRECTION_FACTOR_CM_TO_METER])}
                                        
        return return_dict

    def close(self):
        super(Field, self).close()

    def step(self, actions):
        """
        Actions are w and v velocities of the allies robots
        """

        # SPIN 
        # stepOutSpin = 4
        # if self.step_episode > 12:
        #     if not self.robots_allies[0].isAlive() and not self.robots_allies[0].spin:
        #         if self.step_episode - self.robots_allies[0].last_step_spin > 8:
        #             self.robots_allies[0].last_step_spin = self.step_episode
        #             self.robots_allies[0].spin = True

        #     if self.robots_allies[0].spin == True:
        #         if self.step_episode - self.robots_allies[0].last_step_spin < stepOutSpin:
        #             actions[0] = (actions[0][0], 1.5)
        #             if not self.robots_allies[0].dir_changed:
        #                 actions[0] = (-actions[0][0], actions[0][1])
        #                 self.robots_allies[0].dir_changed = True
        #         else: 
        #             self.robots_allies[0].dir_changed = False
        #             self.robots_allies[0].spin = False

        for robot in range(self.num_allies):
            self.lin_and_ang_speed[robot] = (actions[robot][0]*self.max_v, actions[robot][1]*self.max_w)

        # we use 4 loops to give a time to the simulator reach the desired velocit. Because
        # It doesn't happen imediately
        for _ in range(4):
            super(Field, self).run()
        
        self.step_episode += 1

        return (self.next_step(), self.reward(), self.done())
    
    def reward(self):
        """
        It's a simplified reward, that give +1 if the agent do a goal and
        return -1 reward if the agent receive a goal
        """
        reward = 0

        if self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER > self.x_goal_opponent:
            reward = 10
        elif self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER < self.x_goal_allied:
            reward = -10  
        else:

            w_move = 0.2
            w_ball_grad = 0.8
            w_energy = 2e-4
            w_time = 2e-2

            # Calculate ball potential
            grad_ball_potential = self.__ball_grad()
            # Calculate Move ball
            move_reward = self.__move_reward()
            # Calculate Energy penalty
            energy_penalty = self.__energy_penalty()
            # Calculate allignment robotball ballgoal
            proj_robotball_ballgoal = self.__robotball_ballgoal()

            reward = (1/(1+(w_time*(time.time() - self.init_time_episode))))*(w_move * move_reward + w_ball_grad * grad_ball_potential + \
                        w_energy * energy_penalty) 
            
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
    
    def __ball_grad(self):
        '''Calculate ball potential gradient
        Difference of potential of the ball in time_step seconds.
        '''
        # Calculate ball potential
        length_cm = self.ground.length
        half_lenght = (self.ground.length*CORRECTION_FACTOR_CM_TO_METER / 2.0)\
            + self.ground.goal_depth*CORRECTION_FACTOR_CM_TO_METER

        # distance to defence
        dx_d = (half_lenght + self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER)
        # distance to attack
        dx_a = (half_lenght - self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER)
        dy = (self.ball.body.position[1]*CORRECTION_FACTOR_CM_TO_METER)

        dist_1 = -math.sqrt(dx_a ** 2 + 2 * dy ** 2)
        dist_2 = math.sqrt(dx_d ** 2 + 2 * dy ** 2)
        ball_potential = ((dist_1 + dist_2) / length_cm - 1) / 2

        grad_ball_potential = 0
        # Calculate ball potential gradient
        # = actual_potential - previous_potential
        if self.previous_ball_potential is not None:
            diff = ball_potential - self.previous_ball_potential
            grad_ball_potential = np.clip(diff * 3 / TIME_STEP,
                                          -5.0, 5.0)

        self.previous_ball_potential = ball_potential

        return grad_ball_potential

    def __move_reward(self):
        '''Calculate Move to ball reward
        Cosine between the robot vel vector and the vector robot -> ball.
        This indicates rather the robot is moving towards the ball or not.
        '''

        ball = np.array([self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER, self.ball.body.position[1]*CORRECTION_FACTOR_CM_TO_METER])
        robot = np.array([self.robots_allies[0].body.position[0]*CORRECTION_FACTOR_CM_TO_METER,
                          self.robots_allies[0].body.position[1]*CORRECTION_FACTOR_CM_TO_METER])
        robot_vel = np.array([math.cos(self.robots_allies[0].body.angle),
                              math.sin(self.robots_allies[0].body.angle)])
        robot_ball = ball - robot
        robot_ball = robot_ball/np.linalg.norm(robot_ball)

        move_reward = np.dot(robot_ball, robot_vel)

        move_reward = np.clip(move_reward / 0.4, -5.0, 5.0)
        return move_reward

    def __robotball_ballgoal(self):
        '''Calculate Robot and ball Move to goal reward
        Cosine between the robotball vel vector and the vector ball -> goal.
        This indicates rather the robot and ball is moving towards the goal or not.
        '''

        # ball = np.array([self.ball.body.position[0]*CORRECTION_FACTOR_CM_TO_METER, self.ball.body.position[1]*CORRECTION_FACTOR_CM_TO_METER])
        # robot = np.array([self.robots_allies[0].body.position[0]*CORRECTION_FACTOR_CM_TO_METER,
        #                   self.robots_allies[0].body.position[1]*CORRECTION_FACTOR_CM_TO_METER])
        # robot_vel = np.array([math.cos(self.robots_allies[0].body.angle),
        #                       math.sin(self.robots_allies[0].body.angle)])
        # robot_ball = ball - robot
        # robot_ball = robot_ball/np.linalg.norm(robot_ball)

        # move_reward = np.dot(robot_ball, robot_vel)

        # move_reward = np.clip(move_reward / 0.4, -5.0, 5.0)
        # return move_reward
        return 0

    def __energy_penalty(self):
        '''Calculates the energy penalty'''

        linearVelocity = math.sqrt(self.robots_allies[0].body.linearVelocity[0]**2 + self.robots_allies[0].body.linearVelocity[1]**2)
        en_penalty_1 = abs(linearVelocity*CORRECTION_FACTOR_CM_TO_METER)
        en_penalty_2 = abs(self.robots_allies[0].body.angularVelocity)
        energy_penalty = - (en_penalty_1 + en_penalty_2)
        return energy_penalty