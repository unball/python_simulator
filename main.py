#!/usr/bin/env python3
#   -*- coding: utf-8 -*-
#      @author: Hiago dos Santos Rabelo (hiagop22@gmail.com)

from field import *
import os

# # -----------------------------------------------------------------------------
# os.environ['SDL_VIDEO_CENTERED'] = '1'

# ------------ Set the icon for the application ------------
directory = os.getcwd()
logo = pygame.image.load(directory + '/images/UnBall.png') 
pygame.display.set_icon(logo)

# ----------------------------------------------------------

render = True

def config_render(render):
    """
    Disable or enable render mode. If you just want train a NN without see the training, call this function
    passing False as an argument.
    """
    if not render:
        render = False

