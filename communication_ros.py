#!/usr/bin/env python3

import rospy
from python_simulator.msg import comm_msg
from python_simulator.msg import VisionMessage
from objects_on_field.physics_engine import (motor_voltage_to_wheels_speed, 
                                            wheels_speeds_to_robots_speeds)


class RunRos(object):
    def __init__(self, *arg, **kargs):
        self.vision_message = VisionMessage()
        self.ang_and_lin_speed = [(0,0), (0,0), (0,0), (0,0), (0,0), (0,0)]

        print('simulator node started....')

        rospy.init_node('simulator_node', anonymous=True)
        self.pub = rospy.Publisher('vision_output_topic', VisionMessage, queue_size=1)
        self.sub = rospy.Subscriber('radio_topic', comm_msg, self.callback)

        self.num_allies = 0    
        self.num_opponents = 0

    def callback(self, data):
        print(data)
        wheels = [motor_voltage_to_wheels_speed(data.MotorA[x], data.MotorB[x]) 
                                                     for x in range(self.num_allies)]
        self.ang_and_lin_speed = [wheels_speeds_to_robots_speeds(wheels[x][0], wheels[x][1]) 
                                                                for x in range(self.num_allies)]

    def update(self, pos_robots, pos_ball):
        self.update_vision_message(pos_robots, pos_ball)
        self.pub.publish(self.vision_message)

    def update_vision_message(self, pos_robots, pos_ball):
        for x in range(self.num_allies):
            self.vision_message.x[x] = pos_robots[x][0][0]
            self.vision_message.y[x] = pos_robots[x][0][1]
            self.vision_message.th[x] = pos_robots[x][1]
            self.vision_message.found[x] = True

        self.vision_message.ball_x = pos_ball[0]
        self.vision_message.ball_y = pos_ball[1]

"""
if paused:
for i in range(3):
    speeds.linear_vel[i] = 0
    speeds.angular_vel[i] = 0
    motors.MotorA[i] = 0
    motors.MotorB[i] = 0

"""