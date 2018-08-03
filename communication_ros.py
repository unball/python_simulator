#!/usr/bin/env python3

import rospy
from communication.msg import robots_speeds_msg

from std_msgs.msg import Float32

saida = Float32()

def add_communication_with_system(cls):
    """Class decorator function 
    """
    class Run_ros(cls):
        def __init__(self, *arg, **kargs):
            cls.__init__(self, *arg, **kargs)            
            cls.run_communication = self.run()
            cls.run_communication()

        def run(cls):
            try: 
                rospy.init_node('Simulator')
                pub1 = rospy.Publisher('robots_speeds', Float32, queue_size=1)

                rate = rospy.Rate(30)   
                while not rospy.is_shutdown():
                    pub1.publish(saida)
                    rate.sleep()
                    cls.game()
            except rospy.ROSInterruptException:
                pass

    
    return Run_ros