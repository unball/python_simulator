#!usr/bin/env python3
#-*- coding: utf-8 -*-
"""
    @author: Hiago dos Santos (hiagop22@gmail.com)
    @description: This is the main-menu 
    Based on Pablo Pizarro R. pygame-menu:
    https://github.com/ppizarror/pygame-menu/blob/master/example2.py

The MIT License (MIT)
Copyright 2017-2018 Pablo Pizarro R. @ppizarror

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

# Import pygame and libraries
from pygame.locals import *
from random import randrange
import os
import pygame

# Import pygameMenu
import pygameMenu
from pygameMenu.locals import *

#ABOUT = ['PygameMenu {0}'.format(pygameMenu.__version__),
#         'Author: {0}'.format(pygameMenu.__author__),
#         PYGAMEMENU_TEXT_NEWLINE,
#         'Email: {0}'.format(pygameMenu.__email__)]
ABOUT = ['Python Simulation',
         'Author: Hiago dos Santos Rabelo',
         PYGAMEMENU_TEXT_NEWLINE,
         'Email: hiagop22@gmail.com']


COLOR_BACKGROUND = (3, 63, 118) 
COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (255, 255, 255)
FPS = 60.0
MENU_BACKGROUND_COLOR = (228, 55, 36)
WINDOW_SIZE = (640, 480)

# -----------------------------------------------------------------------------
# Init pygame
pygame.init()
os.environ['SDL_VIDEO_CENTERED'] = '1'

# Create pygame screen and objects
surface = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption('Python Simulation')
clock = pygame.time.Clock()
dt = 1 / FPS

# Global variables
DIFFICULTY = ['EASY']


# -----------------------------------------------------------------------------

def change_difficulty(d):
    """
    Change difficulty of the game.
    
    :return: 
    """
    print ('Selected difficulty: {0}'.format(d))
    DIFFICULTY[0] = d


def random_color():
    """
    Return random color.
    
    :return: Color tuple
    """
    return randrange(0, 255), randrange(0, 255), randrange(0, 255)


def play_function(difficulty, font):
    """
    Main game function
    
    :param difficulty: Difficulty of the game
    :param font: Pygame font
    :return: None
    """
    difficulty = difficulty[0]
    assert isinstance(difficulty, str)

    if difficulty == 'EASY':
        f = font.render('Playing as baby', 1, COLOR_WHITE)
    elif difficulty == 'MEDIUM':
        f = font.render('Playing as normie', 1, COLOR_WHITE)
    elif difficulty == 'HARD':
        f = font.render('Playing as god', 1, COLOR_WHITE)
    else:
        raise Exception('Unknown difficulty {0}'.format(difficulty))

    # Draw random color and text
    bg_color = random_color()
    f_width = f.get_size()[0]

    # Reset main menu and disable
    # You also can set another menu, like a 'pause menu', or just use the same
    # main_menu as the menu that will check all your input.
    main_menu.disable()
    main_menu.reset(1)

    while True:

        # Clock tick
        clock.tick(60)

        # Application events
        playevents = pygame.event.get()
        for e in playevents:
            if e.type == QUIT:
                exit()
            elif e.type == KEYDOWN:
                if e.key == K_ESCAPE and main_menu.is_disabled():
                    main_menu.enable()

                    # Quit this function, then skip to loop of main-menu on line 217
                    return

        # Pass events to main_menu
        main_menu.mainloop(playevents)

        # Continue playing
        surface.fill(bg_color)
        surface.blit(f, ((WINDOW_SIZE[0] - f_width) / 2, WINDOW_SIZE[1] / 2))
        pygame.display.flip()


def main_background():
    """
    Function used by menus, draw on background while menu is active.
    
    :return: None
    """
    surface.fill(COLOR_BACKGROUND)


# -----------------------------------------------------------------------------
# PLAY MENU
play_menu = pygameMenu.Menu(surface,
                            bgfun=main_background,
                            color_selected=COLOR_WHITE,
                            font=pygameMenu.fonts.FONT_BEBAS,
                            font_color=COLOR_BLACK,
                            font_size=30,
                            menu_alpha=100,
                            menu_color=MENU_BACKGROUND_COLOR,
                            menu_height=int(WINDOW_SIZE[1] * 0.6),
                            menu_width=int(WINDOW_SIZE[0] * 0.6),
                            onclose=PYGAME_MENU_DISABLE_CLOSE,
                            option_shadow=False,
                            title='Play menu',
                            window_height=WINDOW_SIZE[1],
                            window_width=WINDOW_SIZE[0]
                            )
# When pressing return -> play(DIFFICULTY[0], font)
play_menu.add_option('Start', play_function, DIFFICULTY,
                     pygame.font.Font(pygameMenu.fonts.FONT_FRANCHISE, 30))
play_menu.add_selector('Select difficulty', [('Easy', 'EASY'),
                                             ('Medium', 'MEDIUM'),
                                             ('Hard', 'HARD')],
                       onreturn=None,
                       onchange=change_difficulty)
play_menu.add_option('Return to main menu', PYGAME_MENU_BACK)

# ABOUT MENU
about_menu = pygameMenu.TextMenu(surface,
                                 bgfun=main_background,
                                 color_selected=COLOR_WHITE,
                                 font=pygameMenu.fonts.FONT_BEBAS,
                                 font_color=COLOR_BLACK,
                                 font_size_title=30,
                                 font_title=pygameMenu.fonts.FONT_8BIT,
                                 menu_color=MENU_BACKGROUND_COLOR,
                                 menu_color_title=COLOR_WHITE,
                                 menu_height=int(WINDOW_SIZE[1] * 0.6),
                                 menu_width=int(WINDOW_SIZE[0] * 0.6),
                                 onclose=PYGAME_MENU_DISABLE_CLOSE,
                                 option_shadow=False,
                                 text_color=COLOR_BLACK,
                                 text_fontsize=20,
                                 title='About',
                                 window_height=WINDOW_SIZE[1],
                                 window_width=WINDOW_SIZE[0]
                                 )
for m in ABOUT:
    about_menu.add_line(m)
about_menu.add_line(PYGAMEMENU_TEXT_NEWLINE)
about_menu.add_option('Return to menu', PYGAME_MENU_BACK)

# MAIN MENU
main_menu = pygameMenu.Menu(surface,
                            bgfun=main_background,
                            color_selected=COLOR_WHITE,
                            font=pygameMenu.fonts.FONT_BEBAS,
                            font_color=COLOR_BLACK,
                            font_size=30,
                            menu_alpha=100,
                            menu_color=MENU_BACKGROUND_COLOR,
                            menu_height=int(WINDOW_SIZE[1] * 0.6),
                            menu_width=int(WINDOW_SIZE[0] * 0.6),
                            onclose=PYGAME_MENU_DISABLE_CLOSE,
                            option_shadow=False,
                            title='Main menu',
                            window_height=WINDOW_SIZE[1],
                            window_width=WINDOW_SIZE[0]
                            )
main_menu.add_option('Play', play_menu)
main_menu.add_option('About', about_menu)
main_menu.add_option('Quit', PYGAME_MENU_EXIT)

# -----------------------------------------------------------------------------
# Main loop
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


#from resolution import *
import os
from field import * # Contain the Classes of the objects in field and tkinter
try:
    from PIL import Image as Img
    from PIL import ImageTk

    # Checking the version of python in other use the respective module
    version_py = sys.version_info[0] < 3
    if version_py:
        from Tkinter import *
        import ttk
    else:
        from tkinter import *
        from tkinter import ttk
except ImportError as excessao:
    print(excessao)
    sys.exit()


class Simulator(object):
    """UnBall Simulator"""
    def __init__(self, instance):
        self.directory = os.getcwd()
        self.directory
        self.instance = instance
        self.instance.title('UnBall Simulator')                               # Defining the title
        self.img1 = PhotoImage(file=self.directory + '/src/python_simulator/images/UnBall.png')                      # Defining the ...
        self.instance.tk.call('wm', 'iconphoto', self.instance._w, self.img1) # ... simulator ico

        SW = instance.winfo_screenwidth()
        SH = instance.winfo_screenheight()
        self.instance.geometry("%dx%d" % (SW,SH))
        #self.instance.geometry(RESOLUTION)                                    # Defining the resolution
        self.instance.configure(background=BACKGROUND_COLOR)                  # Defining the background color

        self.fonte_head1 = ('Sans Serif', 16) 
        self.fonte_head2 = ('Times New Roman', 22) 
        self.fonte_button = ('Sans Serif', 14, 'bold')

        self.declare_widgets_init()
        self.run_init_window()

    def declare_widgets_init(self):
        """This method declares the frames for first window"""

        # Background frame
        self.bg_frame = Frame(self.instance, bg=BACKGROUND_COLOR)

        # Setting up the distance between the labels text and other elements
        self.texts_first_window = (30, 1) # above:30   bellow:1 

        # Setting up the logo
        img = Img.open(self.directory + '/src/python_simulator/images/UnBall.png')
        img.thumbnail(LOGO_SIZE, Img.ANTIALIAS)
        img = ImageTk.PhotoImage(img)
        self.logo = Label(self.bg_frame)
        self.logo['image'] = img
        self.logo.image = img
        self.logo['bg'] = BACKGROUND_COLOR

        # Setting up the text of the teams
        self.text_allies = Label(self.bg_frame, text = 'Allies', bg = BACKGROUND_COLOR)
        self.text_allies['font'] = self.fonte_head1

        self.text_opponents = Label(self.bg_frame, text = 'Opponents'
                                    , bg = BACKGROUND_COLOR)
        self.text_opponents['font'] = self.fonte_head1

        # Setting up the combobox of the teams
        self.n_allies = StringVar()                        
        self.n_allies_box = ttk.Combobox(self.bg_frame, width=8
                                          , textvariable=self.n_allies) 
        self.n_allies_box['values'] = (1, 2, 3)
        self.n_allies_box['state'] = 'readonly'
        self.n_allies_box['font'] = self.fonte_head1
        self.n_allies_box.current(2) 

        self.n_opponents = StringVar()                        
        self.n_opponents_box = ttk.Combobox(self.bg_frame, width=8
                                             , textvariable=self.n_opponents) 
        self.n_opponents_box['values'] = (1, 2, 3)
        self.n_opponents_box['state'] = 'readonly'
        self.n_opponents_box['font'] = self.fonte_head1
        self.n_opponents_box.current(2) 

        # Setting up the text of the color team
        self.text_team_color = Label(self.bg_frame, text='Team Color'
                                    , bg=BACKGROUND_COLOR)
        self.text_team_color['font'] = self.fonte_head1

        # Setting up the combobox of the team_color
        self.color_team = StringVar()                        
        self.color_team_box = ttk.Combobox(self.bg_frame, width=8
                                           , textvariable=self.color_team) 
        self.color_team_box['values'] = ('Yellow', 'Blue')
        self.color_team_box['state'] = 'readonly'
        self.color_team_box['font'] = self.fonte_head1
        self.color_team_box.current(1) 

        # Setting up the button of the run_simulation
        self.run_simulation_button = Button(self.bg_frame, width=12
                                           , command=self.run_simulation)
        self.run_simulation_button['text'] = 'Run Simulation'
        self.run_simulation_button['font'] = self.fonte_head1

        # Setting up the button of the run_gui
        self.run_gui_button = Button(self.bg_frame, width=12, command=self.run_gui)
        self.run_gui_button['text'] = 'Run GUI'
        self.run_gui_button['font'] = self.fonte_head1

        # Setting up the button of the run_gui
        self.config_button = Button(self.bg_frame, width=12, command=self.config)
        self.config_button['text'] = 'Config'
        self.config_button['font'] = self.fonte_head1

    def run_init_window(self):
        """This method initialize the widgets of the first window"""

        self.bg_frame.pack()
        self.logo.grid(row=0, column=0, pady=self.texts_first_window, sticky=N+E+W+S, 
                        columnspan=2)
        self.text_allies.grid(row=1, column=0, pady=self.texts_first_window, sticky=N+E+W+S)
        self.text_opponents.grid(row=1, column=1, pady=self.texts_first_window, sticky=N+E+W+S)
        self.n_allies_box.grid(row=2, column=0, padx=3)
        self.n_opponents_box.grid(row=2, column=1, padx=3)
        self.text_team_color.grid(row=3, column=0, columnspan=2, pady=self.texts_first_window)
        self.color_team_box.grid(row=4, column=0, columnspan=2, pady=2)
        self.run_simulation_button.grid(row=6, column=0, columnspan=2, pady=(70, 1))
        self.run_gui_button.grid(row=7, column=0, columnspan=2, pady=2)
        self.config_button.grid(row=8, column=0, columnspan=2, pady=2)

    def run_simulation(self):
        """This method destroys the labels from first_windows and create the 
        necessary labels for show the simulator"""

        self.destroy_labels_init_window()
        self.declare_widgets_simulation()

        self.frame1.pack(side=LEFT, anchor=NW, expand=True, fill='both')
        self.frame2.pack(side=LEFT, anchor=CENTER, pady = 20, expand=True, fill='both')

        self.title_simulator.pack(ipadx=20)
        self.text_field.pack(ipady=20)
        self.side_team_box.pack()

        self.instance.update() # Refreshing the pygame 

        self.subframe3.pack(anchor=CENTER)#, expand=True, fill='both')
        self.play_pause.pack(side=LEFT, pady=BUTTON_SEPARATOR, padx=20, ipadx=15)
        self.quit.pack(side=LEFT, pady=BUTTON_SEPARATOR, padx=20, ipadx=15)

        ### Field inserted in center - Pygame ###
        # Tell pygame's SDL window which window ID to use
        os.environ['SDL_WINDOWID'] = str(self.frame2.winfo_id())
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        self.field_simul = Field(frame=self.frame2, color_team=self.color_team.get(), 
                             n_allies=self.n_allies.get(), 
                             n_opponents=self.n_opponents.get())

    def destroy_labels_init_window(self):
        """Destroy the labels from first_window"""

        self.bg_frame.destroy()
        self.text_allies.destroy()
        self.text_opponents.destroy()
        self.n_allies_box.destroy()
        self.n_opponents_box.destroy()
        self.text_team_color.destroy()
        self.color_team_box.destroy()
        self.run_simulation_button.destroy()
        self.run_gui_button.destroy()
        self.config_button.destroy()

    def declare_widgets_simulation(self):
        """This method defines all widgets necessaries for run simulator"""

        # Frames 
        self.frame1 = Frame(self.instance, bg=BACKGROUND_COLOR)
        self.frame2 = Frame(self.instance, width=(FIELD_W * PPM), height=(FIELD_H * PPM), 
                            bg=BACKGROUND_COLOR)
        self.subframe3 = Frame(self.frame1, bg=BACKGROUND_COLOR)

        ### Text inserted in left ###

        # Setting up the title
        self.title_simulator = Label(self.frame1, text='SIMULATION', font=self.fonte_head2)
        self.title_simulator['bg'] = BACKGROUND_COLOR

        # Setting up the title
        self.text_field = Label(self.frame1, text='Field Side', font=self.fonte_head1)
        self.text_field['bg'] = BACKGROUND_COLOR

        # Setting up the combobox of the team_color
        self.field_side = StringVar()                        
        self.side_team_box = ttk.Combobox(self.frame1, width=8
                                         , textvariable=self.field_side) 
        self.side_team_box['values'] = ('Left', 'Right')
        self.side_team_box['state'] = 'readonly'
        self.side_team_box['font'] = self.fonte_head1
        self.side_team_box.current(1) 

        # Play/Pause button
        img_play = Img.open(self.directory + '/src/python_simulator/images/play.png') 
        img_play = img_play.resize((50, 50), Img.ANTIALIAS) 
        img_play = ImageTk.PhotoImage(img_play) # Turning possible use the image in GUI

        self.play_pause = Button(self.subframe3, fg=BACKGROUND_COLOR, bg='white'
                                , activeforeground='white')
        self.play_pause['image'] = img_play
        self.play_pause.image = img_play

        # Quit button
        self.quit = Button(self.subframe3, activeforeground='white', height=2, width=2)
        self.quit['command'] = self.get_out_simulator
        self.quit['bg'] = 'brown'
        self.quit['fg'] = 'white'
        self.quit['font'] = self.fonte_button
        self.quit['text'] = 'Quit'

    def get_out_simulator(self):
        """Finish the simulator and come back to main menu (init window)"""

        self.destroy_widgets_simulation()
        self.declare_widgets_init()
        self.run_init_window()

    def destroy_widgets_simulation(self):
        """Destroy the widgets from simulator """

        self.frame1.destroy()
        self.frame2.destroy()

        self.title_simulator.destroy()
        self.text_field.destroy()
        self.side_team_box.destroy()
        self.field_simul.destroy()

    def run_gui(self):
        pass

    def config(self):
        pass
