#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Based on Chris Campbell's tutorial from iforce2d.net:
http://www.iforce2d.net/b2dtut/top-down-car
"""

from pygame_framework.framework import *
from pygame_framework.backends.pygame_framework import *
from pygame_framework.physics_engine import BodyMovingOnGround
import math

class Ball(PygameFramework):
    def __init__(self, world, color, density=1, position=(20 , 20)):
        super(Ball, self).__init__()

        self.body = world.CreateDynamicBody(
            fixtures=b2FixtureDef(shape=b2CircleShape(radius=2), 
                                  density=1.0,
                                  friction=0.2,
                                  restitution=0.1),
            bullet=True,
            position=position)
        self.body.userData = {'obj': self}
        self.color = color

    def update(self):
        self.update_friction()

    @property
    def forward_velocity(self):
        body = self.body
        current_normal = body.GetWorldVector((0, 1))
        return current_normal.dot(body.linearVelocity) * current_normal

    def update_friction(self):
        #impulse = -self.lateral_velocity * self.body.mass
        #if impulse.length > self.max_lateral_impulse:
        #    impulse *= self.max_lateral_impulse / impulse.length

        #self.body.ApplyLinearImpulse(self.current_traction * impulse,
        #                             self.body.worldCenter, True)

        #aimp = 0.1 * self.current_traction * \
        #Remove AngularMoment
        aimp = 0.1 * \
            self.body.inertia * -self.body.angularVelocity
        self.body.ApplyAngularImpulse(aimp, True)

        #Remove LinearMoment
        current_forward_normal = self.forward_velocity
        current_forward_speed = current_forward_normal.Normalize()

        drag_force_magnitude = -6 * current_forward_speed
        self.body.ApplyForce(drag_force_magnitude * current_forward_normal,
                             self.body.worldCenter, True)


class Walls(PygameFramework):
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

    def __init__(self, friction_modifier):
        self.friction_modifier = friction_modifier

class Lines_on_ground(PygameFramework):
    """
    Lines that there are on field
    """
    def __init__(self, world, color):
        super(Lines_on_ground, self).__init__()
        #pygame.draw.circle(self.screen, (255,255,255), (0,20), 40, 5)
        world.rendered.DrawArc((int((FIELD_W/2) - (BIG_FIELD_RADIUS*PPM)), 
                        int((FIELD_H/2) - (BIG_FIELD_RADIUS*PPM)), BIG_FIELD_RADIUS*PPM*2, 
                        BIG_FIELD_RADIUS*PPM*2), 0, 8, b2Color(0.3, 0.4, 0.1))

        #world.renderer.DrawCircle((0,0), 20, b2Color(0, 1, 0.6))
        """
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
        """
        self.color = color



class Robots(PygameFramework):
    length_robot = 7.5 # cm
    pos_init = {
    'allie': [70, 47],
    'oppon':  [104, 47]
    }

    def __init__(self,world, color = '', team=''):
        self.body = world.CreateDynamicBody(position=self.__class__.pos_init[team], 
                                            angle=1.5)
        self.body.CreatePolygonFixture(box=(__class__.length_robot/2,__class__.length_robot/2), 
                                  density=1, friction=0.3, restitution=0.2)
        self.body.userData = {'obj': self}
        self.color = color
        
        self.update_pos_init(team)

    def update_pos_init(self, team):
        pos = __class__.pos_init[team]
        pos[1] += 20
        __class__.pos_init[team] = pos

    @classmethod
    def reset_values_pos(cls):
        cls.pos_init['allie'] = [70, 47]
        cls.pos_init['oppon'] = [104, 47]

    def __del__(self):
        self.__class__.reset_values_pos()

    def Keyboard(self, key):
        if key == Keys.K_d:
            self.platform.type = b2_dynamicBody
        elif key == Keys.K_s:
            self.platform.type = b2_staticBody
        elif key == Keys.K_k:
            self.platform.type = b2_kinematicBody
            self.platform.linearVelocity = (-self.speed, 0)
            self.platform.angularVelocity = 0


    def convert_vel_wheels_for_lin_and_ang(self, vel_road_LR = [0, 0]):
        vel_lin = Vector((vel_road_LR[0] + vel_road_LR[1]) / 2, self.item.angle)
        
        self.vel_lin = [vel_lin.coord_x, vel_lin.coord_y]
        self.vel_ang = (vel_road_LR[1] - vel_road_LR[0]) / self.__class__.length_robot


class Robot(PygameFramework):
    vertices = [(3.75, 0.0),
                (3.75, 7.5),
                (-3.75, 7.5),
                (-3.75, 0.0),
                ]

    tire_anchors = [(-3.0, 0.75),
                    (3.0, 0.75),
                    (-3.0, 8.50),
                    (3.0, 8.50),
                    ]

    def __init__(self, world, vertices=None,
                 tire_anchors=None, density=0.1, position=(0, 0),
                 **tire_kws):
        if vertices is None:
            vertices = Robot.vertices

        self.body = world.CreateDynamicBody(position=position)
        self.body.CreatePolygonFixture(vertices=vertices, density=density)
        self.body.userData = {'obj': self}

        self.tires = [BodyMovingOnGround(self, **tire_kws) for i in range(4)]

        if tire_anchors is None:
            anchors = Robot.tire_anchors

        joints = self.joints = []
        for tire, anchor in zip(self.tires, anchors):
            j = world.CreateRevoluteJoint(bodyA=self.body,
                                          bodyB=tire.body,
                                          localAnchorA=anchor,
                                          # center of tire
                                          localAnchorB=(0, 0),
                                          enableMotor=False,
                                          maxMotorTorque=1000,
                                          enableLimit=True,
                                          lowerAngle=0,
                                          upperAngle=0,
                                          )

            tire.body.position = self.body.worldCenter + anchor
            joints.append(j)

    def update(self, keys, hz):
        for tire in self.tires:
            tire.update_friction()

        for tire in self.tires:
            tire.update_drive(keys)

        # control steering
        lock_angle = math.radians(40.)
        # from lock to lock in 0.5 sec
        turn_speed_per_sec = math.radians(160.)
        turn_per_timestep = turn_speed_per_sec / hz
        desired_angle = 0.0

        if 'left' in keys:
            desired_angle = lock_angle
        elif 'right' in keys:
            desired_angle = -lock_angle

        front_left_joint, front_right_joint = self.joints[2:4]
        angle_now = front_left_joint.angle
        angle_to_turn = desired_angle - angle_now

        # TODO fix b2Clamp for non-b2Vec2 types
        if angle_to_turn < -turn_per_timestep:
            angle_to_turn = -turn_per_timestep
        elif angle_to_turn > turn_per_timestep:
            angle_to_turn = turn_per_timestep

        new_angle = angle_now + angle_to_turn
        # Rotate the tires by locking the limits:
        front_left_joint.SetLimits(new_angle, new_angle)
        front_right_joint.SetLimits(new_angle, new_angle)
