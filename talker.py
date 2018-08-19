#!usr/bin/env python2

import rospy
from python_simulator.msg import robots_speeds_msg
from python_simulator.msg import wheels_speeds_msg
from python_simulator.msg import VisionMessage

saida = wheels_speeds_msg()

def talker():
    pub = rospy.Publisher('radio_topic', wheels_speeds_msg, queue_size=1)
    rospy.init_node('talker_publisher', anonymous=True)
    rate = rospy.Rate(1)
    i= [[0, 0 , 0], [0, 0, 0]]
    while not rospy.is_shutdown():
        saida.right_vel = i[0]
        saida.left_vel = i[1]
        pub.publish(saida)
        i[0][0] = 1
        i[1][0] = -1
        rate.sleep()

if __name__ == '__main__' :
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

# em um terminal executar roscore
# em um terminal executar python2 talker.py
# em outro terminal executar com rostopic echo /talker