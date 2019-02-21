#!usr/bin/env python3
#
#   -*- coding: utf-8 -*-
#      @author: Hiago dos Santos Rabelo (hiagop22@gmail.com)
# @description: Based on Chris Campbell's tutorial from iforce2d.net:
#               'http://www.iforce2d.net/b2dtut/top-down-car'


import math as m
from constants import *

class PhysicsBall(object):

    def __init__(self, body):
        self.body = body

    def update_friction(self):
        self.body.linearDamping = 0.2
        self.body.angularDamping = 0.2


class PhysicsRobot(object):

    def __init__(self, robot, position):

        self.body = robot
        self.current_traction = 1
        self.ground_areas = []

    @property
    def forward_velocity(self):
        body = self.body
        current_normal = body.GetWorldVector((1, 0))
        return current_normal.dot(body.linearVelocity) * current_normal

    @property
    def lateral_velocity(self):
        body = self.body

        right_normal = body.GetWorldVector((0, 1))
        return right_normal.dot(body.linearVelocity) * right_normal


    def update_friction(self):
        aimp = 0.1 * self.current_traction * \
            self.body.inertia * -self.body.angularVelocity
        self.body.ApplyAngularImpulse(aimp, True)

        current_forward_normal = self.forward_velocity
        current_forward_speed = current_forward_normal.Normalize()

        drag_force_magnitude =  0 * current_forward_speed
        self.body.ApplyForce(drag_force_magnitude * current_forward_normal,
                             self.body.worldCenter, True)

        impulse = -self.lateral_velocity * self.body.mass
        
        self.body.ApplyLinearImpulse(self.current_traction * impulse,
                                     self.body.worldCenter, True)

    def update_drive(self, desired_linear_velocity):

        # find the current speed in the forward direction
        current_forward_normal = self.body.GetWorldVector((1, 0))
        print(current_forward_normal)
        current_speed = self.forward_velocity.dot(current_forward_normal)

        # apply necessary force
        linear_velocity_difference = desired_linear_velocity - current_speed
        desired_inpulse = 0.5 * self.body.mass * linear_velocity_difference
        
        self.body.ApplyLinearImpulse(desired_inpulse * current_forward_normal,
                             self.body.worldCenter, True)

    def update_turn(self, desired_angular_velocity):

        angular_velocity_difference = desired_angular_velocity - self.body.angularVelocity
        desired_inpulse = 0.5 * self.body.inertia * angular_velocity_difference
    
        self.body.ApplyAngularImpulse(desired_inpulse, True)


    def update_traction(self):
        #if not self.ground_areas:
            self.current_traction = 1
        #else:
        #    self.current_traction = 0
        #    mods = [ga.friction_modifier for ga in self.ground_areas]

        #    max_mod = max(mods)
        #    if max_mod > self.current_traction:
        #        self.current_traction = max_mod