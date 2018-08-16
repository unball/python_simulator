#!/usr/bin/env python3

import rospy
#from python_simulator.msg import robots_speeds_msg
#from msg import wheels_speeds_msg
#from msg import VisionMessage

### Para teste ### 
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
            # self.pub1 = rospy.Publisher('vision_output_topic', VisionMessage, queue_size=1)
            # rospy.Subscriber('radio_topic',wheels_speeds_msg,self.update_speeds)
            self.pub1 = rospy.Publisher('robots_speeds', Float32, queue_size=1)

            #rate = rospy.Rate(30)
            self.j = 0
            self.iterador()   

        def iterador(self):
                if rospy.is_shutdown():               # Finish the communication if ctrl+c
                    return                            # is pressed
                self.j += 1
                self.frame.after(2000, self.iterador) # 2000 milliseconds = 2 seconds
                self.pub1.publish(self.j)

        def update_speeds(self, data):
            pass

    return Run_ros