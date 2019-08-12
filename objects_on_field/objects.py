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
    

class Ball(PhysicsBall):
    def __init__(self, world, color, density=1, position=(0, 0)):
        self.body = world.CreateDynamicBody(
            fixtures=b2FixtureDef(shape=b2CircleShape(radius=2.135), 
                                  friction=0.1,
                                  restitution=0.1),
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

    def __init__(self, world, color, friction=0.8):
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


    def __init__(self, world, friction_modifier = ''):
        self.world = world

    def update(self):
        self.world.renderer.DrawCircle(self.world.renderer.to_screen(b2Vec2(0,0)),
                                         20, WHITE)
        for x in range(len(Ground.pos_lines)):
            self.world.renderer.DrawSegment(
                self.world.renderer.to_screen(b2Vec2(Ground.pos_lines[x][0])), 
                self.world.renderer.to_screen(b2Vec2(Ground.pos_lines[x][1])), 
                WHITE)

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
    position = [('', 0), ('', 0), ('', 0),('', 0), ('', 0)]
    color_line = [(255,0,0), (0,255,0), (255,255,255), (255,0,255), (0,255,255)]

    def __init__(self, world, color, side, position=(0, 0)):

        self.main_world = world
        self.body = world.CreateDynamicBody(position=(position[0], Robot.position[position[1]][1]))
        self.body.CreatePolygonFixture(box=Robot.dimensions,
                                      density=(MASS_ROBOT/
                                        (((self.__class__.dimensions[0]*2)**3)*(10**(-2)))))
        self.body.userData = {'obj': self}
        self.body.bullet = True
        
        if position[1] == 0:
            if side == 'left':
                self.body.angle = m.pi/2
            else:
                self.body.angle = - m.pi/2
        elif side == 'left':
            self.body.angle = 0
        else:
            self.body.angle = - m.pi
        self.color = YELLOW if color else BLUE
        self.num_robot = position[1]

        super(Robot, self).__init__(self.body, position)

    def update_colors(self):
        ball = self.main_world.renderer.DrawSolidCircle(
                                self.main_world.renderer.to_screen(self.body.position),
                                3, (-m.cos(self.body.angle), -m.sin(self.body.angle)), self.color,
                                color_line=Robot.color_line[self.num_robot])


    def update(self, desired_velocity, hz): 
        """
        desired_velocity[0] = angular velocity
        desired_velocity[1] = linear velocity
        """
        super(Robot, self).update_friction()
        if desired_velocity:
            super(Robot, self).update_turn(desired_velocity[0])
            super(Robot, self).update_drive(desired_velocity[1])

