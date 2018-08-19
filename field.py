#!usr/bin/env python3
#-*- coding: utf-8 -*-
"""
    @author: Hiago dos Santos (hiagop22@gmail.com)
    @description: This module describes the Class Field and others necessary classes
    used into the Field class.
    Based on 'https://github.com/pybox2d/pybox2d/blob/master/examples/simple/simple_02.py'
"""

import rospy
import math 
from constants import *
import sys
from communication_ros import *

try:
    from PIL import Image as Img
    from PIL import ImageTk
    import pygame # Turn possible the relationship between tkinter and Box2D
    import Box2D  # The main library
    # Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
    from Box2D.b2 import (world, polygonShape, circleShape, staticBody, dynamicBody)

    # Checking the version of python in other use the respective module
    version_py = sys.version_info[0] < 3
    if version_py:
        from Tkinter import *
        import ttk
    else:
        from tkinter import *
        from tkinter import ttk
except ImportError as excessao:
    print(excessao)
    sys.exit()


class Null(object):
    def __init__(self):
        pass

class Vector(object):
    def __init__(self, angle, norm):
        self.angle = angle
        self.norm = norm

    @property
    def coord_x(self):
        return self.angle*math.cos(self.norm)

    @property
    def coord_y(self):
        return self.angle*math.sin(self.norm)


class Ball(object):
    def __init__(self):
        self.item = ''
        self.name = 'ball'
        self.pos_init  = (174/2, 134/2) # (x, y)
        self.color = ORANGE 
        self.pos_xy = ''     
        

class Robot(object):
    length_robot = 7.5 # cm

    def __init__(self):
        self.item = Null()
        self.item.angle = 0
        self.pos_xy = ''
        self.vel_ang = ''
        self.vel_lin = ''

    def convert_vel_wheels_for_lin_and_ang(self, vel_road_LR = [0, 0]):
        vel_lin = Vector((vel_road_LR[0] + vel_road_LR[1]) / 2, self.item.angle)
        
        self.vel_lin = [vel_lin.coord_x, vel_lin.coord_y]
        self.vel_ang = (vel_road_LR[1] - vel_road_LR[0]) / self.__class__.length_robot
                

class Walls(object):
    def __init__(self):
        self.item = ''
        self.name = 'wall'
        self.pos_and_length = (((168, 109.5), (6,22.5)), ((173, 67), (1,20)),
                            ((168, 24.5), (6,22.5)), ((87, 1), (87,1)), 
                            ((6, 24.5), (6,22.5)), ((1, 67), (1,20)),
                            ((6, 109.5), (6,22.5)), ((87, 133), (87,1)))
        self.color = UnBall_blue



# Class decorator, see: 
# https://pt.stackoverflow.com/questions/23628/como-funcionam-decoradores-em-python
# book: Luiz Eduardo Borges. Python para desenvolvedores, 2º ed. pg: 139

@add_communication_with_system 
class Field(object):
    """ *args - argumentos sem nome (lista)
        **kargs - argumentos com nome (dicionário)
    """
    def __init__(self, *args, **kargs):

        self.frame=kargs.pop('frame')

        # --- pygame setup ---

        self.screen = pygame.display.set_mode((int(FIELD_W), int(FIELD_H)) )

        # --- pybox2d world setup --- 

        self.world = world(gravity=(0, 0), doSleep=True)

        # CAUTION: staticBody and Polygon create a body centered on position and 
        #          box defines half-size; 
        # i.e :
        # box=(2, 40) => box with width: 4 and length: 80 
        
        self.pos_init_rob = {
            'allie': ((70, 87), (70, 67), (70, 47)),
            'oppon': ((104, 87), (104, 67), (104, 47))
        }

        self.walls = Walls()
        self.create_walls()
        
        self.ball = Ball()
        self.create_ball()

        self.colors = {
            'wall': self.walls.color,
            'ball': self.ball.color,
            'allie': (127, 127, 127),
            'oppon': (127, 127, 12) 
        }

        if kargs.pop('color_team') == 'Yellow':
            self.robot_allie = [YELLOW, int(kargs.pop('n_allies')), 'allie']
        else:
            self.robot_allie = [BLUE, int(kargs.pop('n_allies')), 'allie']
        self.robot_allie = self.create_robots(self.robot_allie)

        self.robot_oppo = [OPPON_COLOR, int(kargs.pop('n_opponents')), 'oppon']
        self.robot_oppo = self.create_robots(self.robot_oppo)
        
        pygame.display.init()
        self.bg_color = pygame.Color(0,0,0)
        self.ball_color = ORANGE
        
        def my_draw_polygon(polygon, body, fixture):
            vertices = [(body.transform * v) * PPM for v in polygon.vertices]
            vertices = [(v[0], FIELD_H - v[1]) for v in vertices]
            pygame.draw.polygon(self.screen, self.colors[body.userData], vertices)
        polygonShape.draw = my_draw_polygon

        def my_draw_circle(circle, body, fixture):  
            position = body.transform * circle.pos * PPM
            position = (position[0], FIELD_H - position[1])
            pygame.draw.circle(self.screen, self.colors[body.userData], [int(
                x) for x in position], int(circle.radius * PPM))
            # Note: Python 3.x will enforce that pygame get the integers it requests,
            #       and it will not convert from float.
        circleShape.draw = my_draw_circle
        self.i = 20
        self.game()

    def game(self):
        self.screen.fill(self.bg_color)
        self.draw_lines_and_circle() # draw lines in field

        # Make Box2D simulate the physics of our world for one step.
        # Draw the world
        for body in self.world.bodies:
            for fixture in body.fixtures:
                fixture.shape.draw(body, fixture)

        # Make Box2D simulate the physics of our world for one step.
        self.world.Step(TIME_STEP, 10, 10)
        pygame.display.flip()

        self.frame.after(1, self.game)

    def create_walls(self):
        for x in self.walls.pos_and_length:
            wall = self.world.CreateStaticBody(
            position=x[0],
            shapes=polygonShape(box=x[1]) 
            )
            wall.userData = self.walls.name

    def create_ball(self):
        body = self.world.CreateDynamicBody(position=self.ball.pos_init)
        body.userData = self.ball.name
        self.ball.item = body
        body.CreateCircleFixture(radius=BALL_RADIUS, density=1, 
                                                  friction=0.3, restitution=0.8)
    
    def create_robots(self, robot):
        color = robot[0]
        n = robot[1]
        name = robot[2]
        robot = [Robot() for x in range(n)]
        for x in range(n):
            body = self.world.CreateDynamicBody(position=self.pos_init_rob[name][x], 
                                                angle=1.5)
            body.userData = name
            robot[x].item = body
            body.CreatePolygonFixture(box=(ROBOT_W/2, ROBOT_H/2), density=1, 
                                                  friction=0.3, restitution=0.2)

        return robot

    def draw_lines_and_circle(self):
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

        # grande lua
        pygame.draw.arc(self.screen, WHITE, (int((FIELD_W/2) - (BIG_FIELD_RADIUS*PPM)), 
                        int((FIELD_H/2) - (BIG_FIELD_RADIUS*PPM)), BIG_FIELD_RADIUS*PPM*2, 
                        BIG_FIELD_RADIUS*PPM*2), 0, 8)

        for x in lines:
            pygame.draw.line(self.screen, WHITE, x[0], x[1])

    def destroy(self):
        """Finish the pygame when quit button is pressed"""
        pygame.quit()
