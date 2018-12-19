#!usr/bin/env python3
#-*- coding: utf-8 -*-
"""
    @author: Hiago dos Santos (hiagop22@gmail.com)
    @description: This module describes the Class Field and others necessary classes
    used into the Field class.
    Based on 'https://github.com/pybox2d/pybox2d/blob/master/examples/simple/simple_02.py'
"""

import math 
from constants import *
import sys
from communication_ros import *
from objects_on_field.objects import (Ball, Walls, Robot)

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

# Class decorator, see: 
# https://pt.stackoverflow.com/questions/23628/como-funcionam-decoradores-em-python
# book: Luiz Eduardo Borges. Python para desenvolvedores, 2º ed. pg: 139

#@add_communication_with_system 
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

        self.walls = Walls(self.world, UnBall_blue)
        self.ball = Ball(self.world, BALL_RADIUS, ORANGE)

        self.colors = {
            'wall': self.walls.color,
            'ball': self.ball.color,
            'allie': (127, 127, 127),
            'oppon': (127, 127, 12) 
        }

        collor_allie = YELLOW if kargs.pop('color_team') == 'Yellow' else BLUE
        n_allies = int(kargs.pop('n_allies'))
        n_oppo = int(kargs.pop('n_opponents'))

        self.robot_allie = [Robot(self.world, collor_allie, 'allie') for v in range(n_allies)]
        self.robot_oppo = [OPPON_COLOR, n_oppo, 'oppon']
        self.robot_oppo = [Robot(self.world, collor_allie, 'oppon') for v in range(n_oppo)]
        
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
        #self.i = 20
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
        """Finish the pygame and bodies into world when quit button is pressed"""
        for body in self.world.bodies:
            self.world.DestroyBody(body)
        Robot.reset_values_pos()
        pygame.quit()
