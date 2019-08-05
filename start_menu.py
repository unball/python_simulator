#!/usr/bin/env python3
#   -*- coding: utf-8 -*-
#      @author: Hiago dos Santos Rabelo (hiagop22@gmail.com)
# @description: This is the main-menu 
# Based on Pablo Pizarro R. pygame-menu:
# https://github.com/ppizarror/pygame-menu/blob/master/example2.py
#
# The MIT License (MIT)
# Copyright 2017-2018 Pablo Pizarro R. @ppizarror
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the Software
# is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


# Import pygame and libraries
from pygame.locals import *
from random import randrange
from field import *
import os
import pygame
from constants import *

# Import pygameMenu
import pygameMenu
from pygameMenu.locals import *

ABOUT = ['Python Simulation',
         TEXT_NEWLINE,
         'Property: UnBall ',
         'Email: equipe.unball@gmail.com',
         TEXT_NEWLINE,
         'Developer: Hiago dos Santos Rabelo',
         'Email: hiagop22@gmail.com'
         ]
HELP = ['>>MENU',
        'Press UP/DOWN to navigate on menu',
        'Press LEFT/RIGHT to change the option',
        'Press ENTER to enter in a topic',
        TEXT_NEWLINE,
        '>>SIMULATION:',
        'Press SPACE to Pause/Play into Simulation',
        'Press ESC to return to main menu',
        'Click on a object and keep pressed to move it']


COLOR_BACKGROUND = (3, 63, 118) 
COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (255, 255, 255)
FPS = 60.0
MENU_BACKGROUND_COLOR = (228, 55, 36)
WINDOW_SIZE = (int(FIELD_H*ZOOM), int(FIELD_W*ZOOM))

# -----------------------------------------------------------------------------
# Init pygame
pygame.init()
os.environ['SDL_VIDEO_CENTERED'] = '1'

# Create pygame screen and objects
directory = os.getcwd()
logo = pygame.image.load(directory + '/src/python_simulator/images/UnBall.png') 
pygame.display.set_icon(logo)

surface = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption('UnBall - Python Simulation')

clock = pygame.time.Clock()
dt = 1 / FPS

# Global variables
num_allies = 0
num_opponents = 0
team_color = 0  # blue   = 0
                # yellow = 1
publish_topic = 'vision_output_topic'
field_side = 'left'

# -----------------------------------------------------------------------------
def change_num_allies(d, i):
    print ('Selected number allies: {0}'.format(d))
    global num_allies
    num_allies = d[1]

def change_num_opponents(d, i):
    print ('Selected number opponents: {0}'.format(d))
    global num_opponents
    num_opponents = d[1]

def change_team_color(d, i):
    print ('Selected number opponents: {0}'.format(d))
    global team_color
    team_color = d[1]

def change_field_side(d, i):
    print ('Selected field side: {0}'.format(d))
    global field_side
    field_side = d[1]

# def change_publish_topic(d):
#     print ('Selected publish topic: {0}'.format(d))
#     global publish_topic
#     publish_topic = d

def play_function():
    main_menu.disable()
    main_menu.reset(1)
    print(num_allies)
    print(num_opponents)
    print(team_color)
    Field(num_allies, num_opponents, team_color, field_side, publish_topic)
    main_menu.enable()
    

def main_background():
    """
    Function used by menus, draw on background while menu is active.
    
    :return: None
    """
    surface.fill(COLOR_BACKGROUND)


# -----------------------------------------------------------------------------
# RUN SIMULATION MENU
run_simulation = pygameMenu.Menu(surface,
                            bgfun=main_background,
                            color_selected=COLOR_WHITE,
                            font=pygameMenu.font.FONT_BEBAS,
                            font_color=COLOR_BLACK,
                            font_size=30,
                            menu_alpha=100,
                            menu_color=MENU_BACKGROUND_COLOR,
                            menu_height=int(WINDOW_SIZE[1] * 0.8),
                            menu_width=int(WINDOW_SIZE[0] * 0.8),
                            onclose=pygameMenu.events.DISABLE_CLOSE,
                            option_shadow=False,
                            title='Run Simulation',
                            window_height=WINDOW_SIZE[1],
                            window_width=WINDOW_SIZE[0]
                            )

run_simulation.add_option('Start', play_function)
run_simulation.add_selector('Number allies', [('0', 0),
                                             ('1', 1),
                                             ('2', 2),
                                             ('3', 3),
                                             ('4', 4),
                                             ('5', 5)],
                        onreturn=None,
                        onchange=change_num_allies)
run_simulation.add_selector('Number opponents', [('0', 0),
                                             ('1', 1),
                                             ('2', 2),
                                             ('3', 3),
                                             ('4', 4),
                                             ('5', 5)],
                        onreturn=None,
                        onchange=change_num_opponents)               
run_simulation.add_selector('Team color', [('Blue', 0),
                                             ('Yellow', 1)],
                        onreturn=None,
                        onchange=change_team_color)                                          
run_simulation.add_selector('Field side', [('left', 'left'),
                                             ('right', 'right')],
                        onreturn=None,
                        onchange=change_field_side)                                          
run_simulation.add_option('Return to main menu', pygameMenu.events.BACK)

# CONFIG MENU
# setting = pygameMenu.Menu(surface,
#                             bgfun=main_background,
#                             color_selected=COLOR_WHITE,
#                             font=pygameMenu.font.FONT_BEBAS,
#                             font_color=COLOR_BLACK,
#                             font_size=30,
#                             menu_alpha=100,
#                             menu_color=MENU_BACKGROUND_COLOR,
#                             menu_height=int(WINDOW_SIZE[1] * 0.8),
#                             menu_width=int(WINDOW_SIZE[0] * 0.8),
#                             onclose=pygameMenu.events.DISABLE_CLOSE,
#                             option_shadow=False,
#                             title='Setting',
#                             window_height=WINDOW_SIZE[1],
#                             window_width=WINDOW_SIZE[0]
#                             )
# setting.add_selector('Publish topic', [('VisionMessage', 'vision_output_topic')],
#                         onreturn=None,
#                         onchange=change_publish_topic)
# setting.add_option('Return to main menu', pygameMenu.events.BACK)

# ABOUT MENU
about_menu = pygameMenu.TextMenu(surface,
                                 bgfun=main_background,
                                 color_selected=COLOR_WHITE,
                                 font=pygameMenu.font.FONT_BEBAS,
                                 font_color=COLOR_BLACK,
                                 font_size_title=30,
                                 font_title=pygameMenu.font.FONT_8BIT,
                                 menu_color=MENU_BACKGROUND_COLOR,
                                 menu_color_title=COLOR_WHITE,
                                 menu_height=int(WINDOW_SIZE[1] * 0.6),
                                 menu_width=int(WINDOW_SIZE[0] * 0.6),
                                 onclose=pygameMenu.events.DISABLE_CLOSE,
                                 option_shadow=False,
                                 text_color=COLOR_BLACK,
                                 title='About',
                                 window_height=WINDOW_SIZE[1],
                                 window_width=WINDOW_SIZE[0]
                                 )
for m in ABOUT:
    about_menu.add_line(m)
about_menu.add_line(TEXT_NEWLINE)
about_menu.add_option('Return to menu', pygameMenu.events.BACK)

# HELP MENU
help_menu = pygameMenu.TextMenu(surface,
                                 bgfun=main_background,
                                 color_selected=COLOR_WHITE,
                                 font=pygameMenu.font.FONT_BEBAS,
                                 font_color=COLOR_BLACK,
                                 font_size_title=30,
                                 font_title=pygameMenu.font.FONT_8BIT,
                                 menu_color=MENU_BACKGROUND_COLOR,
                                 menu_color_title=COLOR_WHITE,
                                 menu_height=int(WINDOW_SIZE[1] * 0.8),
                                 menu_width=int(WINDOW_SIZE[0] * 0.8),
                                 onclose=pygameMenu.events.DISABLE_CLOSE,
                                 option_shadow=False,
                                 text_color=COLOR_BLACK,
                                 title='Help',
                                 window_height=WINDOW_SIZE[1],
                                 window_width=WINDOW_SIZE[0]
                                 )
for m in HELP:
    help_menu.add_line(m)
help_menu.add_line(TEXT_NEWLINE)
help_menu.add_option('Return to menu', pygameMenu.events.BACK)

# MAIN MENU
main_menu = pygameMenu.Menu(surface,
                            bgfun=main_background,
                            color_selected=COLOR_WHITE,
                            font=pygameMenu.font.FONT_BEBAS,
                            font_color=COLOR_BLACK,
                            font_size=30,
                            menu_alpha=100,
                            menu_color=MENU_BACKGROUND_COLOR,
                            menu_height=int(WINDOW_SIZE[1] * 0.8),
                            menu_width=int(WINDOW_SIZE[0] * 0.8),
                            onclose=pygameMenu.events.DISABLE_CLOSE,
                            option_shadow=False,
                            title='Python Simulator',
                            window_height=WINDOW_SIZE[1],
                            window_width=WINDOW_SIZE[0]
                            )
main_menu.add_option('Play', run_simulation)
# main_menu.add_option('Setting', setting)
main_menu.add_option('Help', help_menu)
main_menu.add_option('About', about_menu)
main_menu.add_option('Quit', pygameMenu.events.EXIT)

# -----------------------------------------------------------------------------
# Main loop
def start_menu():
    while True:

        # Tick
        clock.tick(60)

        # Application events
        events = pygame.event.get()
        for event in events:
            if event.type == QUIT:
                exit()

        # Main menu
        main_menu.mainloop(events)

        # Flip surface
        pygame.display.flip()
