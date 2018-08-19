#!/usr/bin/env python3

import rospy
from python_simulator.msg import robots_speeds_msg
from python_simulator.msg import wheels_speeds_msg
from python_simulator.msg import VisionMessage

n_robots = 3


def add_communication_with_system(cls):
    """Class decorator function"""
    class Run_ros(cls):
        def __init__(self, *arg, **kargs):
            try:
                super().__init__(*arg, **kargs)            
                self.run_communication()
            except rospy.ROSInterruptException:
                pass

        def run_communication(self):
            rospy.init_node('Simulator')
            self.pub1 = rospy.Publisher('vision_output_topic', VisionMessage, queue_size=1)
            #rospy.Subscriber('radio_topic', wheels_speeds_msg, self.update_speeds)

            #rate = rospy.Rate(30) está esse valor mesmo ? 
            self.vision_message = VisionMessage()
            self.iterador()   
        
        def iterador(self):
            if rospy.is_shutdown():               
                return                            

            self.update_output_message()
            self.pub1.publish(self.vision_message)

            self.frame.after(2000, self.iterador) # 2000 milliseconds = 2 seconds

        def update_output_message(self):

            self.update_positions_because_the_walls()

            self.vision_message.x = [self.robot_allie[x].pos_xy[0] for x in range(n_robots)] 
            self.vision_message.x += [self.robot_oppo[x].pos_xy[0] for x in range(n_robots)]

            self.vision_message.y = [self.robot_allie[x].pos_xy[1] for x in range(n_robots)] 
            self.vision_message.y += [self.robot_oppo[x].pos_xy[1] for x in range(n_robots)]

            self.vision_message.th = [self.robot_allie[x].item.angle for x in range(n_robots)]
            self.vision_message.th += [self.robot_oppo[x].item.angle for x in range(n_robots)]

            self.vision_message.ball_x, self.vision_message.ball_y = self.ball.pos_xy 

            print(self.vision_message)

        def update_speeds(self, data):
            self.convert_vel_weels_for_vel_ang_lin(data)   

            for x in range(n_robots):
                self.robot_allie[x].item.position += self.vel_lin[x]
                self.robot_allie[0].item.angle += self.vel_ang[x]
                print(self.vel_ang[x])

        def update_positions_because_the_walls(self):

            for x in range(n_robots):
                self.robot_allie[x].pos_xy = self.robot_allie[x].item.position - (12, 2)
                self.robot_oppo[x].pos_xy = self.robot_oppo[x].item.position - (12, 2)
                self.ball.pos_xy = self.ball.item.position - (12, 2)

    return Run_ros


