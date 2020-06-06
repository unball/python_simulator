# %% [code]
#!usr/bin/env python3
#-*- coding: utf-8 -*-
"""
    Created on 06-04-2020 / 14:06:49
    @author: Hiago dos Santos
    @e-mail: hiagop22@gmail.com
    @description: This script show how to use the field simulator and the respective necessary
    methods into the field class that you'll need to train a neural network based on the enviroment.
"""

# %% [code]
# Install necessary dependences
import os
try:
    os.system('pip install pygame')
    os.system('pip install box2d-py')
    os.system('pip install Box2D')
except:
    pass

# Necessary for make main script import simulator
try:
    from distutils.dir_util import copy_tree

    fromDirectory = '../input/simulator'
    toDirectory = ''

    copy_tree(fromDirectory, toDirectory)
except:
    pass
# %% [code]

from field import *

try:
    # Show is there's some device available
    print(os.environ["DISPLAY"])
except:
    # If any device is avaiable, this bellow line avoid errors been raised     
    os.environ["SDL_VIDEODRIVER"] = "dummy"

MAX_EPISODES = 1
MAX_STEPS_ONE_GAME = 50 # steps

steps_game = 0  # Use it to bound the maximum steps during a one_game_duration
                # IMPORTANT: FPS doesn't mean a faster simulation, just a more
                # precise and smoth transition betwen the frames !!!!!
                # To improve the time simulation find a method to improve the MAIN LOOP bellow, such as
                # run the simulation without graphics, use a GPU or parallel processing !!!!

# Field is the enviroment and into there you'll find all necessary methods as reset, step, render and close.
env = Field(num_allies=5, num_opponents=5, team_color='blue', allied_field_side='left', render=True, cloud=False)

# Main Loop
for episode in range(MAX_EPISODES):
    state = env.reset()
    print(state)
    env.keep_running = True
    steps_game = 0
    # env.keep_running return False if some event such as quit buttom is pressed
    # this atribute avoid errors when using render mode
    # End the game if the simulation
    # You also can remove env.keep_runing bellow and put it above inside an if to break all iterations, instead
    # break just one episode
    while env.keep_running and steps_game < MAX_STEPS_ONE_GAME: 
        # we use 4 loops to give a time to the simulator reach the desired velocit. Beacause
        # It doesn't happen imediately
        for x in range(4):
            # insert here angular and linear velocities from the allie robots
            next_state, reward, done = env.step([(0,3.14) for _ in range(5)])
            # print(next_state)
            # print(reward)
            # print(done)
            # state = next_state

        if done: break

        steps_game += 1
        # print(steps_game)

    # Deallocate memory allocated for objects, because the of Box2D run using C++, and it allocate memory. So
    # it's necessary deallocate this memory allocated before
    env.close()