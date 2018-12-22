#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Based on Chris Campbell's tutorial from iforce2d.net:
http://www.iforce2d.net/b2dtut/top-down-car
"""

from pygame_framework.framework import *
from pygame_framework.backends.pygame_framework import *
from pygame_framework.physics_engine import *
from constants import *
import math

class Ball(PhysicsBall):
    def __init__(self, world, color, density=1, position=(20 , 20)):
        self.body = world.CreateDynamicBody(
            fixtures=b2FixtureDef(shape=b2CircleShape(radius=2), 
                                  density=1.0,
                                  friction=0.1,
                                  restitution=0.1),
            bullet=True,
            position=position)
        self.body.userData = {'obj': self}
        self.color = color
        super(Ball, self).__init__(self.body)

    def update(self):
        self.update_friction()
    
class Walls(object):
    """
    See images/Field.jpeg for more details
    """
    pos_and_length = (((81, 42.5), (6,22.5)), ((86, 0), (1,20)),
                        ((81, -42.5), (6,22.5)), ((0, -66), (87,1)), 
                        ((-81, -42.5), (6,22.5)), ((-86, 0), (1,20)),
                        ((-81, 42.5), (6,22.5)), ((0, 66), (87,1)))
    
    def __init__(self, world, color):
        super(Walls, self).__init__()

        self.body = ''
        for x in self.__class__.pos_and_length:
            wall = world.CreateStaticBody(
            position=x[0],
            fixtures=b2FixtureDef(friction=0.8,
                                  shape=b2PolygonShape(box=x[1]))
            )
            wall.userData = {'obj': self}
        self.color = color

class Ground(object):
    """
    An area on the ground that the robots can run over
    """

    def __init__(self, world, friction_modifier = ''):
        self.world = world

    def update(self):
        self.world.renderer.DrawCircle(self.world.renderer.to_screen(b2Vec2(0,0)),
                                         20, WHITE)
        self.world.renderer.DrawSegment(self.world.renderer.to_screen(b2Vec2(0,65)), 
                                        self.world.renderer.to_screen(b2Vec2(0,-65)), 
                                        WHITE)


class Robot(PhysicsRobot):
    dimensions = (3.60, 3.60)

    def __init__(self, world, max_forward_speed=100.0,
                 max_backward_speed=-20, max_drive_force=150,
                 turn_torque=1500, max_lateral_impulse=20,
                 density=0.1, position=(0, 0)):

        self.main_world = world
        self.body = world.CreateDynamicBody(position=position)
        self.body.CreatePolygonFixture(box=Robot.dimensions, density=density)
        self.body.userData = {'obj': self}

        super(Robot, self).__init__(self.body, max_forward_speed, max_backward_speed,
                                    max_drive_force, turn_torque, max_lateral_impulse, 
                                    density, position)

    def update(self, keys, hz):
        super(Robot, self).update_friction()
        super(Robot, self).update_drive(keys)
        super(Robot, self).update_turn(keys)


"""
class Lines_on_ground(object):
    
    def __init__(self, world, color):
        super(Lines_on_ground, self).__init__()
        #PhysicsEnginecle(self.screen, (255,255,255), (0,20), 40, 5)
        world.rendered.DrawArc((int((FIELD_W/2) - (BIG_FIELD_RADIUS*PPM)), 
                        int((FIELD_H/2) - (BIG_FIELD_RADIUS*PPM)), BIG_FIELD_RADIUS*PPM*2, 
                        BIG_FIELD_RADIUS*PPM*2), 0, 8, b2Color(0.3, 0.4, 0.1))

        #world.renderer.DrawCircle((0,0), 20, b2Color(0, 1, 0.6))
        
        lines = (((FIELD_W/2, 0), (FIELD_W/2, FIELD_H)),                # linha de meio campo
                 ((FIELD_W-(12*PPM), 0), (FIELD_W-(12*PPM), FIELD_H)),  # linha de meta direita
                 ((12*PPM, 0), (12*PPM, FIELD_H)),                      # linha de meta esquerda
                 ((12*PPM, (FIELD_H/2) - (35*PPM)), (27*PPM, (FIELD_H/2) - (35*PPM))),  # linha de
                 ((12*PPM, (FIELD_H/2) + (35*PPM)), (27*PPM, (FIELD_H/2) + (35*PPM))),  # tiro penal 
                 ((27*PPM, (FIELD_H/2) - (35*PPM)), (27*PPM, (FIELD_H/2) + (35*PPM))),  # esquerda
                 ((FIELD_W-(12*PPM), (FIELD_H/2) - (35*PPM)), (FIELD_W-(27*PPM), (FIELD_H/2) - (35*PPM))), # linha de
                 ((FIELD_W-(12*PPM), (FIELD_H/2) + (35*PPM)), (FIELD_W-(27*PPM), (FIELD_H/2) + (35*PPM))), # tiro penal
                 ((FIELD_W-(27*PPM), (FIELD_H/2) - (35*PPM)), (FIELD_W-(27*PPM), (FIELD_H/2) + (35*PPM))), # direita
                 )
        
        self.color = color

"""