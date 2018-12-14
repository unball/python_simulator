#!/usr/bin/env python3
#-*- coding: utf-8 -*-
"""
    @author: Hiago dos Santos (hiagop22@gmail.com)
    @description: This module describes communication between the Graphical User 
    Interface(GUI) and the UnBall system
"""

#from resolution import *
from simulator import *

def main():
    instance = Tk()
    Simulator(instance)
    instance.mainloop()       
    
if __name__ == '__main__':
    main()