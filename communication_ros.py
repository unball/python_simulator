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
            try:
                cls.__init__(self, *arg, **kargs)            
                self.run_communication()
            except rospy.ROSInterruptException:
                pass

        def run_communication(self):
            rospy.init_node('Simulator')
            self.pub1 = rospy.Publisher('robots_speeds', Float32, queue_size=1)

            #rate = rospy.Rate(30)
            self.iterador()   
            self.j = 0

        def iterador(self):
                if rospy.is_shutdown():               # Finish the communication if ctrl+c
                    return
                self.j += 1
                self.frame.after(2000, self.iterador) # 2 seconds
                self.pub1.publish(self.j)

    return Run_ros