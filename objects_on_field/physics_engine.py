#!usr/bin/env python3
#
#   -*- coding: utf-8 -*-
#      @author: UnBall (equipe.unball@gmail.com)
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

    def __init__(self, robot, max_lateral_impulse, position):

        self.body = robot
        self.current_traction = 1
        self.max_lateral_impulse = max_lateral_impulse
        self.ground_areas = []

    @property
    def forward_velocity(self):
        body = self.body
        current_normal = body.GetWorldVector((1, 0))
        return current_normal.dot(body.linearVelocity) * current_normal

    @property
    def lateral_velocity(self):
        body = self.body

        right_normal = body.GetWorldVector((0, -1))
        return right_normal.dot(body.linearVelocity) * right_normal


    def update_friction(self):
        aimp = 0.1 * self.current_traction * \
            self.body.inertia * -self.body.angularVelocity
        self.body.ApplyAngularImpulse(aimp, True)

        current_forward_normal = self.forward_velocity
        current_forward_speed = current_forward_normal.Normalize()

        drag_force_magnitude = -2 * current_forward_speed
        self.body.ApplyForce(self.current_traction * drag_force_magnitude * current_forward_normal,
                             self.body.worldCenter, True)

        impulse = -self.lateral_velocity * self.body.mass
        if impulse.length > self.max_lateral_impulse:
            impulse *= self.max_lateral_impulse / impulse.length

        self.body.ApplyLinearImpulse(self.current_traction * impulse,
                                     self.body.worldCenter, True)

    def update_drive(self, desired_linear_velocity):

        # find the current speed in the forward direction
        current_forward_normal = self.body.GetWorldVector((1, 0))
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


#max_tics is the maximum number of tics per second the encoder can read. This parameter depends on 
#which microcontroller is used reduction is the motor to wheels reduction. This paramter depends on
#the physical structure of the robot encoder is the encoder resolution. This parameter depends on 
#which motor brand and model is used
def motor_voltage_to_wheels_speed(motorA,motorB,max_tics=(700/0.01),reduction=(1./1.),encoder=512.*19.):
    motorA_rad_per_s = (motorA/255.0)*(max_tics/encoder)*m.pi;
    wheelA = motorA_rad_per_s*reduction;
    motorB_rad_per_s = (motorB/255.0)*(max_tics/encoder)*m.pi;
    wheelB = motorB_rad_per_s*reduction;

    return wheelA, wheelB

#wheel_radius is the wheel radius. It depends on the robot
#robot_lenght is the distance from one wheel to the other. It depends on the robot
def wheels_speeds_to_robots_speeds(wheelA,wheelB,wheel_radius=0.03,robot_lenght=0.075):
    angular_speed = wheel_radius*(wheelA - wheelB)/robot_lenght;
    linear_speed = wheel_radius*(wheelA + wheelB)/2;

    print("angular_system>>>", angular_speed)
    return angular_speed, linear_speed*CORRECTION_FATOR_METER_TO_CM


def actual_axis_to_axis_unball(pos_robots_allies, pos_robots_opponents, pos_ball):
    new_pos_robots_allies = []
    new_pos_robots_opponents = []

    for x in range(len(pos_robots_allies)):
        new_pos_robots_allies.append(((pos_robots_allies[x][0][0] - CENTER_AXIS_X, 
                                      pos_robots_allies[x][0][1] - CENTER_AXIS_Y), 
                                      pos_robots_allies[x][1]))

    for x in range(len(pos_robots_opponents)):
        new_pos_robots_opponents.append(((pos_robots_opponents[x][0][0] - CENTER_AXIS_X, 
                                      pos_robots_opponents[x][0][1] - CENTER_AXIS_Y), 
                                      pos_robots_opponents[x][1]))

    new_pos_ball = (pos_ball[0] - CENTER_AXIS_X, pos_ball[1] - CENTER_AXIS_Y)

    return new_pos_robots_allies, new_pos_robots_opponents, new_pos_ball
