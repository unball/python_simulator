#!/usr/bin/env python3

import rospy
from python_simulator.msg import robots_speeds_msg
from python_simulator.msg import wheels_speeds_msg
from python_simulator.msg import VisionMessage

def add_communication_with_system(cls):
    """Class decorator function 
    """
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
            rospy.Subscriber('radio_topic', wheels_speeds_msg, self.update_speeds)

            #rate = rospy.Rate(30)
            self.j = VisionMessage()
            self.k = [0, 0, 0, 0, 0, 0]
            self.iterador()   

        def iterador(self):
                if rospy.is_shutdown():               # Finish the communication if ctrl+c
                    return                            # is pressed

                self.k[0] += 1.0
                self.j.x = self.k

                self.frame.after(2000, self.iterador) # 2000 milliseconds = 2 seconds
                self.pub1.publish(self.j)

        def update_speeds(self, data):
            pass

    return Run_ros