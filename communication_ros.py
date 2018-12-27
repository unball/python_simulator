#!/usr/bin/env python3

import rospy
from constants import *
from python_simulator.msg import comm_msg
from python_simulator.msg import VisionMessage
from objects_on_field.physics_engine import (motor_voltage_to_wheels_speed, 
                                            wheels_speeds_to_robots_speeds,
                                            actual_axis_to_axis_unball)


class RunRos(object):
    def __init__(self, publish_topic):
        self.vision_message = VisionMessage()
        self.ang_and_lin_speed = [(0,0), (0,0), (0,0), (0,0), (0,0), (0,0)]

        print('simulator node started....')

        rospy.init_node('simulator_node', anonymous=True)
        if publish_topic == 'vision_output_topic':
            self.pub = rospy.Publisher('vision_output_topic', VisionMessage, queue_size=1)
        else:
            pass # Inserir aqui o outro nó para ser publicado caso seja mudado no menu inicial
                 # Obs.: é necessário inserir a outra opção de nó tanto aqui quanto no menu inicial

        self.sub = rospy.Subscriber('radio_topic', comm_msg, self.callback)

        self.num_allies = 0    
        self.num_opponents = 0

    def callback(self, data):
        wheels = [motor_voltage_to_wheels_speed(data.MotorA[x], data.MotorB[x]) 
                                                     for x in range(self.num_allies)]
        self.ang_and_lin_speed = [wheels_speeds_to_robots_speeds(wheels[x][0], wheels[x][1]) 
                                                                for x in range(self.num_allies)]

    def update(self, pos_robots_allies, pos_robots_opponents, pos_ball):
        self.update_vision_message(pos_robots_allies, pos_robots_opponents, pos_ball)
        self.pub.publish(self.vision_message)

    def update_vision_message(self, pos_robots_allies, pos_robots_opponents, pos_ball):
        pos_robots_allies, pos_robots_opponents, pos_ball = actual_axis_to_axis_unball(pos_robots_allies, 
                                                                                       pos_robots_opponents, 
                                                                                       pos_ball)

        for x in range(self.num_allies):
            self.vision_message.x[x] = pos_robots_allies[x][0][0]
            self.vision_message.y[x] = pos_robots_allies[x][0][1]
            self.vision_message.th[x] = pos_robots_allies[x][1]
            self.vision_message.found[x] = True

        for x in range(self.num_opponents):
            self.vision_message.x[x + MAX_ROBOTS_ALLIES] = pos_robots_opponents[x][0][0]
            self.vision_message.y[x + MAX_ROBOTS_ALLIES] = pos_robots_opponents[x][0][1]
            self.vision_message.th[x + MAX_ROBOTS_ALLIES] = pos_robots_opponents[x][1]
            self.vision_message.found[x + MAX_ROBOTS_ALLIES] = True

        self.vision_message.ball_x = pos_ball[0]
        self.vision_message.ball_y = pos_ball[1]

        print(self.vision_message)