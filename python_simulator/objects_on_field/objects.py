#!/usr/bin/env python
#   -*- coding: utf-8 -*-
#      @author: Hiago dos Santos Rabelo (hiagop22@gmail.com)
# Based on Chris Campbell's tutorial from iforce2d.net:
# http://www.iforce2d.net/b2dtut/top-down-car


from pygame_framework.framework import *
from pygame_framework.backends.pygame_framework import *
from objects_on_field.physics_engine import *
from constants import *
import math as m
import numpy as np
    

class Ball(PhysicsBall):
    def __init__(self, world, color, density=1, position=(0, 0)):
        self.body = world.CreateDynamicBody(
            fixtures=b2FixtureDef(shape=b2CircleShape(radius=2.135), 
                                  friction=0.15,
                                  restitution=0.4),
            bullet=True,
            position=position)
        self.body.userData = {'obj': self}
        self.body.mass = MASS_BALL
        self.body.bullet = True
        self.color = color
        super(Ball, self).__init__(self.body)

    def update(self):
        self.update_friction()
    
class Walls(object):
    """
    See images/Field.jpeg for more details
    """
    pos_and_length_walls = (((81, 42.5), (6,22.5)), ((86, 0), (1,20)),
                            ((81, -42.5), (6,22.5)), ((0, -66), (87,1)), 
                            ((-81, -42.5), (6,22.5)), ((-86, 0), (1,20)),
                            ((-81, 42.5), (6,22.5)), ((0, 66), (87,1)))
    vertices_triangle_walls = [[(68, -65), (75, -58), (75, -65)],
                               [(68, 65), (75, 58), (75, 65)],
                               [(-68, 65), (-75, 58), (-75, 65)],
                               [(-68, -65), (-75, -58), (-75, -65)]]

    def __init__(self, world, color, friction=0.15):
        super(Walls, self).__init__()

        self.body = ''
        for x in self.__class__.pos_and_length_walls:
            wall = world.CreateStaticBody(
            position=x[0],
            fixtures=b2FixtureDef(friction=friction,
                                  shape=b2PolygonShape(box=x[1]))
            )
            wall.userData = {'obj': self}
        for vertices in self.__class__.vertices_triangle_walls:
            wall = world.CreateStaticBody(
            fixtures=b2FixtureDef(friction=friction,
                                  shape=b2PolygonShape(vertices=vertices))
            )
            wall.userData = {'obj': self}
        self.color = color

class Ground(object):
    """
    An area on the ground that the robots can run over
    """
    pos_lines = (((0,65),(0, -65)), ((75,20), (75,-20)), ((-75,20), (-75,-20)), 
                ((60,35), (60,-35)), ((-60,35), (-60,-35)), 
                ((75,35), (60, 35)), ((-75,35), (-60, 35)),
                ((75,-35), (60, -35)), ((-75,-35), (-60, -35)))

    def __init__(self, world, friction_modifier = '', draw_field=False):
        self.world = world
        self.draw_field = draw_field
        self.arrow_step = 7
        self.length = 150
        self.goal_depth = 10
        self.width = 130
        if draw_field:
            self.pos_fild = self.calculate_pos_fild()
            self.angle_field = np.zeros(self.pos_fild.shape[0], dtype=float)
            

    def update(self):
        self.world.renderer.DrawCircle(self.world.renderer.to_screen(b2Vec2(0,0)),
                                         20, WHITE)
        for x in range(len(Ground.pos_lines)):
            self.world.renderer.DrawSegment(
                self.world.renderer.to_screen(b2Vec2(Ground.pos_lines[x][0])), 
                self.world.renderer.to_screen(b2Vec2(Ground.pos_lines[x][1])), 
                WHITE)
        
        if self.draw_field:
            self.draw_field_on_ground()

    def calculate_pos_fild(self):
        x = np.arange(-FIELD_H/2, FIELD_H/2, self.arrow_step)
        y = np.arange(-FIELD_W/2, FIELD_W/2, self.arrow_step)
        X,Y = np.meshgrid(x, y)
        XY = np.array([X.flatten(), Y.flatten()]).T

        return XY
    
    def draw_field_on_ground(self):        
        for i in range(len(self.pos_fild)):
            theta = self.angle_field[i]
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            v2_ = self.pos_fild[i] + np.dot(R, np.array([self.pos_fild[i][0]-(self.arrow_step/2),self.pos_fild[i][1]]) - self.pos_fild[i] )
            v1_ = self.pos_fild[i] + np.dot(R, np.array([self.pos_fild[i][0]+(self.arrow_step/2), self.pos_fild[i][1]]) - self.pos_fild[i])
            arrow = self.world.renderer.DrawSegment(
                self.world.renderer.to_screen((
                    v2_
                    )),
                self.world.renderer.to_screen((
                    v1_
                    )) , WHITE)
            arrow = self.world.renderer.DrawPoint(self.world.renderer.to_screen((
                v1_
            )),2, WHITE)
            
            # arrow = self.world.renderer.DrawSegment(
            #     self.world.renderer.to_screen((
            #         self.pos_fild[i] + np.dot(R, (self.pos_fild[i] - np.array([self.pos_fild[i][0]-(self.arrow_step/2),self.pos_fild[i][1]])))
            #         self.pos_fild[i][0]-(self.arrow_step/2),self.pos_fild[i][1]
            #         )),
            #     self.world.renderer.to_screen((
            #         self.pos_fild[i][0]+(self.arrow_step/2), self.pos_fild[i][1]
            #         )) , WHITE)
            # arrow = self.world.renderer.DrawPoint(self.world.renderer.to_screen((
            #     self.pos_fild[i][0]+(self.arrow_step/2), self.pos_fild[i][1]
            # )),2, WHITE)

class Trajectory(object):
    def __init__(self, world, color, size):
        self.world = world
        self.color = color
        self.size = size

    def update(self, pos_x, pos_y):
        # print(pos)
        for x in range(len(pos_x)):
            self.world.renderer.DrawPoint(
                self.world.renderer.to_screen(b2Vec2(pos_x[x]*CORRECTION_FATOR_METER_TO_CM,
                                                     pos_y[x]*CORRECTION_FATOR_METER_TO_CM)),
                                         self.size, self.color)

class Robot(PhysicsRobot):
    dimensions = (3.75, 3.75)
    start_position = [('', 0), ('', 0), ('', 0),('', 0), ('', 0)]
    color_line = [(255,0,0), (0,255,0), (255,255,255), (255,0,255), (0,255,255)]

    def __init__(self, world, color, num_robot, side='left', start_position=(0, 0),angle=0, y_predefined=True):

        self.main_world = world
        self.y_predefined = y_predefined
        self.spin = False
        self.dir_changed = False
        self.last_step_spin = 0
        if self.y_predefined:
            self.body = world.CreateDynamicBody(position=(start_position[0], Robot.start_position[start_position[1]][1]))
        else:
            self.body = world.CreateDynamicBody(position=(start_position[0], start_position[1]))
        self.body.CreatePolygonFixture(box=Robot.dimensions,
                                      density=(MASS_ROBOT/
                                        (((self.__class__.dimensions[0]*2)**3)*(10**(-2)))))
        self.body.userData = {'obj': self}
        self.body.bullet = True
        
        if not self.y_predefined:
            self.body.angle = angle
        else:
            if start_position[1] == 0:
                if side == 'left':
                    self.body.angle = m.pi/2
                else:
                    self.body.angle = - m.pi/2
            elif side == 'left':
                self.body.angle = 0
            else:
                self.body.angle = - m.pi
        self.color = YELLOW if color else BLUE
        self.num_robot = num_robot

        # super(Robot, self).__init__(self.body, start_position)
        super(Robot, self).__init__(self.body)

    def isAlive(self):
        if abs(np.linalg.norm(self.body.linearVelocity)) < .2 and abs(np.linalg.norm(self.body.angularVelocity)) < 2:
            return False
        
        return True

    def update_colors(self):
        ball = self.main_world.renderer.DrawSolidCircle(
                                self.main_world.renderer.to_screen(self.body.position),
                                3, (-m.cos(self.body.angle), -m.sin(self.body.angle)), self.color,
                                color_line=Robot.color_line[self.num_robot])
        

    def update(self, desired_velocity, hz): 
        """
        desired_velocity[0] = linear velocity
        desired_velocity[1] = angular velocity
        """
        super(Robot, self).update_friction()
        if desired_velocity:
            super(Robot, self).update_drive(desired_velocity[0])
            super(Robot, self).update_turn(desired_velocity[1])

