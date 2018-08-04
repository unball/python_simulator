#!/usr/bin/env python3

import rospy
#from communication.msg import robots_speeds_msg

from std_msgs.msg import Float32

saida = Float32()

def add_communication_with_system(cls):
    """Class decorator function 
    """
    class Run_ros(cls):
        def __init__(self, *arg, **kargs):
            cls.__init__(self, *arg, **kargs)            
            try:
                self.run_communication()
            except rospy.ROSInterruptException:
                pass

        def run_communication(self):
            rospy.init_node('Simulator')
            self.pub1 = rospy.Publisher('robots_speeds', Float32, queue_size=1)

            #rate = rospy.Rate(30)
            self.i = 0
            self.iterador()   

        def iterador(self):
                if rospy.is_shutdown():
                    return
                self.frame.after(2000, self.iterador)
                self.pub1.publish(self.i)
                self.i += 1

    return Run_ros