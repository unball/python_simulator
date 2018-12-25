#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from python_simulator.msg import comm_msg

    
def talker():
    msg = comm_msg()

    for x in range(3):
        msg.MotorA[x] = 250
        msg.MotorB[x] = 250

    pub = rospy.Publisher('radio_topic', comm_msg, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
     
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass