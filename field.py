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
#from communication_ros import *
from objects_on_field.objects import (Ball, Walls, Robot, Ground )#Lines_on_ground)
from pygame_framework.physics_engine import *
#from pygame_framework.framework import FrameworkBase
from pygame_framework.framework import *
import math

N_ROBOTS = 3


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
        self.ground = Ground(self.world)
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
    """
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
            if isinstance(obj, PhysicsEngineRobot):
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
    """

    def Step(self, settings):
        for x in range(N_ROBOTS):
            self.car[x].update(self.pressed_keys, settings.hz)

        self.ball.update()
        self.ground.update()
        super(Field, self).Step(settings)

        
        for x in range(N_ROBOTS):
            #tractions = [self.robot.current_traction for tire in self.car[x].tires]
            print(self.car[x].body.position)
        #self.Print('Current tractions: %s' % tractions)
