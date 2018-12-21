#!usr/bin/env python3
#-*- coding: utf-8 -*-
"""
    @author: Hiago dos Santos (hiagop22@gmail.com)
    @description: This module describes the Class Field and others necessary classes
    used into the Field class.
    Based on Chris Campbell's tutorial from iforce2d.net:
    'http://www.iforce2d.net/b2dtut/top-down-car'
"""

from constants import *
import sys
from communication_ros import *
from objects_on_field.objects import (Ball, Walls, Robot, Ground )#Lines_on_ground)
from pygame_framework.physics_engine import BodyMovingOnGround
#from pygame_framework.framework import FrameworkBase
from pygame_framework.framework import *
import math

N_ROBOTS = 3


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
class Field(PygameFramework):
    name = "Python simulator"
    description = "Robot controled by ros"
    #description = "If there is only one robot. Keys: accel = w, reverse = s, left = a, right = d"

    def __init__(self, *args, **kargs):
        super(Field, self).__init__()
        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)

        self.key_map = {Keys.K_w: 'up',
                        Keys.K_s: 'down',
                        Keys.K_a: 'left',
                        Keys.K_d: 'right',
                        }

        # Keep track of the pressed keys
        self.pressed_keys = set()

        # The walls
        walls = Walls(self.world, BLUE)
        self.ball = Ball(self.world, BLUE)
        #boundary = self.world.CreateStaticBody(position=(0, 20))
        #boundary.CreateEdgeChain([(-30, -30),
        #                          (-30, 30),
        #                          (30, 30),
        #                          (30, -30),
        #                          (-30, -30)]
        #                         )

        # A couple regions of differing traction
        self.car = [Robot(self.world, position=(0,10*x)) for x in range(N_ROBOTS)]
        #gnd1 = self.world.CreateStaticBody(userData={'obj': Ground(0.5)})
        #fixture = gnd1.CreatePolygonFixture(
        #    box=(9, 7, (-10, 15), math.radians(20)))
        # Set as sensors so that the car doesn't collide
        #fixture.sensor = True

        #gnd2 = self.world.CreateStaticBody(userData={'obj': Ground(0.2)})
        #fixture = gnd2.CreatePolygonFixture(
        #    box=(9, 5, (5, 20), math.radians(-40)))
        #fixture.sensor = True

        super(Field, self).run()
        

    def Keyboard(self, key):
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.add(key_map[key])
        else:
            super(Field, self).Keyboard(key)

    def KeyboardUp(self, key):
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.remove(key_map[key])
        else:
            super(Field, self).KeyboardUp(key)

    def handle_contact(self, contact, began):
        # A contact happened -- see if a wheel hit a
        # ground area
        fixture_a = contact.fixtureA
        fixture_b = contact.fixtureB

        body_a, body_b = fixture_a.body, fixture_b.body
        ud_a, ud_b = body_a.userData, body_b.userData
        if not ud_a or not ud_b:
            return

        tire = None
        ground_area = None
        for ud in (ud_a, ud_b):
            obj = ud['obj']
            if isinstance(obj, BodyMovingOnGround):
                tire = obj
            elif isinstance(obj, Ground):
                ground_area = obj

        if ground_area is not None and tire is not None:
            if began:
                tire.add_ground_area(ground_area)
            else:
                tire.remove_ground_area(ground_area)

    def BeginContact(self, contact):
        self.handle_contact(contact, True)

    def EndContact(self, contact):
        self.handle_contact(contact, False)

    def Step(self, settings):
        for x in range(N_ROBOTS):
            self.car[x].update(self.pressed_keys, settings.hz)

        self.ball.update()
        super(Field, self).Step(settings)

        
        for x in range(N_ROBOTS):
            tractions = [tire.current_traction for tire in self.car[x].tires]
            print(self.car[x].body.position)
        #self.Print('Current tractions: %s' % tractions)

"""
class Field(object):
    def __init__(self, *args, **kargs):

        self.frame=kargs.pop('frame')

        # --- pygame setup ---

        #self.screen = pygame.display.set_mode((int(FIELD_W), int(FIELD_H)) )

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
        
        
        def my_draw_polygon(polygon, body, fixture):
            vertices = [(body.transform * v) * PPM for v in polygon.vertices]
            vertices = [(v[0], FIELD_H - v[1]) for v in vertices]
            pygame.draw.polygon(self.screen, self.colors[body.userData], vertices)
        polygonShape.draw = my_draw_polygon
        
        def DrawSolidCircle(self, center, radius, axis, color):
        
            radius *= self.zoom
            if radius < 1:
                radius = 1
            else:
                radius = int(radius)

            pygame.draw.circle(self.surface, (color / 2).bytes + [127],
                               center, radius, 0)
            pygame.draw.circle(self.surface, color.bytes, center, radius, 1)
            pygame.draw.aaline(self.surface, (255, 0, 0), center,
                               (center[0] - radius * axis[0],
                                center[1] + radius * axis[1]))

    
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
"""