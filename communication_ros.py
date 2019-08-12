#!/usr/bin/env python3
#   -*- coding: utf-8 -*-
#      @author: Hiago dos Santos Rabelo (hiagop22@gmail.com)
# @description: This module run the Ros publishing in node chosen on start_menu
#               and subscribing from radio_topic.

import rospy
from constants import *
from communication.msg import robots_speeds_msg
from communication.msg import trajectory
from vision.msg import VisionMessage

class RunRos(object):
    def __init__(self, publish_topic):
        self.vision_message = VisionMessage()
        self.ang_and_lin_speed = [(0,0), (0,0), (0,0), (0,0), (0,0), 
                                 (0,0), (0,0), (0,0), (0,0), (0,0)]
        self.trajectory_x = []
        self.trajectory_y = []

        print('simulator node started....')

        rospy.init_node('simulator_node', anonymous=True)
        if publish_topic == 'vision_output_topic':
            self.pub = rospy.Publisher('vision_output_topic', VisionMessage, queue_size=1)
        else:
            pass # Inserir aqui o outro nó para ser publicado caso seja mudado no menu inicial
                 # Obs.: é necessário inserir a outra opção de nó tanto aqui quanto no menu inicial

        self.sub = rospy.Subscriber('robots_speeds', robots_speeds_msg, self.velocity_callback)
        self.sub = rospy.Subscriber('trajectory', trajectory, self.trajectory_callback)

        self.num_allies = 0
        self.num_opponents = 0

    def velocity_callback(self, data):
        # wheels = [motor_voltage_to_wheels_speed(data.MotorA[x], data.MotorB[x])
                                                     # for x in range(self.num_allies)]
        self.ang_and_lin_speed = [(data.angular_vel[x],
                                  data.linear_vel[x]*CORRECTION_FATOR_METER_TO_CM)
                                  for x in range(self.num_allies + self.num_opponents)]
    
    def trajectory_callback(self, data):
        self.trajectory_x = data.trajectory_x
        self.trajectory_y = data.trajectory_y

    def update(self, pos_robots_allies, pos_robots_opponents, pos_ball):
        self.update_vision_message(pos_robots_allies, pos_robots_opponents, pos_ball)
        self.pub.publish(self.vision_message)

    def update_vision_message(self, pos_robots_allies, pos_robots_opponents, pos_ball):                                                                                       # pos_ball)

        for x in range(self.num_allies):
            self.vision_message.x[x] = pos_robots_allies[x][0][0] * (10**-2)
            self.vision_message.y[x] = pos_robots_allies[x][0][1] * (10**-2)
            self.vision_message.th[x] = pos_robots_allies[x][1]
            self.vision_message.found[x] = True

        for x in range(self.num_opponents):
            self.vision_message.x[x + MAX_ROBOTS_ALLIES] = pos_robots_opponents[x][0][0] * (10**-2)
            self.vision_message.y[x + MAX_ROBOTS_ALLIES] = pos_robots_opponents[x][0][1] * (10**-2)
            self.vision_message.th[x + MAX_ROBOTS_ALLIES] = pos_robots_opponents[x][1]
            self.vision_message.found[x + MAX_ROBOTS_ALLIES] = True

        self.vision_message.ball_x = pos_ball[0] * (10**-2)
        self.vision_message.ball_y = pos_ball[1] * (10**-2)

        # print(self.vision_message)
