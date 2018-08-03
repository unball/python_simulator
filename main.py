#!usr/bin/env python3
#-*- coding: utf-8 -*-
"""
    @author: Hiago dos Santos (hiagop22@gmail.com)
    @description: This module describes communication between the Graphical User 
    Interface(GUI) and the UnBall system
"""

#from resolution import *
from simulator import *

"""
#import threading 

class Communic(threading.Thread):
   
    def run(self):
        try:
            rospy.init_node('simulator_node', anonymous=True)
            pub = rospy.Publisher('radio_topic',comm_msg, queue_size=1)
            #rospy.Subscriber('radio_topic', comm_msg, )    

            rate = rospy.Rate(30)

            while not rospy.is_shutdown():
                rate.sleep()

        except rospy.ROSInterruptException:
            pass

"""

# Criar decorador de classe para adicionar o ROS ao simulator


def main():
    instance = Tk()
    Simulator(instance)
    instance.mainloop()       

if __name__ == '__main__':
    main()
    