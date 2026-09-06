import sys
import os
import math
import json
import time
import traceback
from Soccer.Localisation.class_Glob import Glob
from Soccer.Vision.class_Vision_RPI import Vision_RPI
from Soccer.Localisation.class_Local import *
from Soccer.strategy import Player
from Soccer.Motion.class_Motion_real import Motion_real
from Soccer.config_paths import init_param_read_path, repo_root_str

SIMULATION = 5
current_work_directory = repo_root_str(os.getcwd())

try:
    from button_test import Button_Test
    labels = [[],
              ['forward_left', 'goalkeeper', 'basketball', 'kick_test'],
              [],
              ['forward_center', 'penalty_Shooter'],
              ['forward_right', 'penalty_Goalkeeper', 'run_test', 'jump_test']]
    os.system("espeak -ven-m1 -a200 'Choose Role'")
    button = Button_Test(labels)
    first_pressed_button = button.wait_for_button_pressing(message ="'Choose role'")

    with open(init_param_read_path(current_work_directory, "Real/Real_landmarks.json"), "r") as f:
        landmarks = json.loads(f.read())
    subrole = ' '
    if first_pressed_button == 'forward_center':
        role = 'forward'
        initial_coord = [-0.45, 0, 0]
    if first_pressed_button == 'forward_right':
        role = 'forward'
        subrole = ' Rignt'
        x = -(landmarks['FIELD_LENGTH'] / 2 - 0.87)
        y = -landmarks['FIELD_WIDTH'] / 2
        initial_coord = [x, y, math.pi/2]
    if first_pressed_button == 'forward_left':
        role = 'forward'
        subrole = ' Left'
        x = -(landmarks['FIELD_LENGTH'] / 2 - 0.87)
        y = landmarks['FIELD_WIDTH'] / 2
        initial_coord = [x, y, -math.pi/2]
    if first_pressed_button == 'penalty_Shooter':
        role = 'penalty_Shooter'
        initial_coord = [0.2, 0, 0]
    if first_pressed_button == 'penalty_Goalkeeper':
        role = 'penalty_Goalkeeper'
        initial_coord = [-landmarks['FIELD_LENGTH'] / 2, 0, 0]
    if first_pressed_button == 'goalkeeper':
        role = 'goalkeeper'
        initial_coord = [-landmarks['FIELD_LENGTH'] / 2, 0, 0]
    if first_pressed_button == 'run_test':
        role = 'run_test'
        initial_coord = [0.0, 0.0, 0]
    if first_pressed_button == 'jump_test':
        role = 'jump_test'
        initial_coord = [0.0, 0.0, 0]
    if first_pressed_button == 'basketball':
        role = 'dance'
        initial_coord = [0.0, 0.0, 0]
    if first_pressed_button == 'kick_test':
        role = 'kick_test'
        initial_coord = [0.0, 0.0, 0]
    glob = Glob(SIMULATION, current_work_directory, particles_number = 100, event_type = 'Robocup')
    glob.pf_coord = initial_coord
    if (role == 'forward' or role == 'penalty_Shooter' or role == 'penalty_Goalkeeper' or role == 'goalkeeper' or role == 'kick_test'):
        glob.neural_vision_enable()
        glob.neural_vision = True
    glob.role = role
    vision = Vision_RPI(glob)
    motion = Motion_real(glob, vision)
    motion.falling_Flag = 0
    if role == 'run_test':
        labels = [[],['side_step_left', 'rotation_left'], [], ['short_run', 'long_run', 'spot_run', 'head_tilt_calibration'], ['side_step_right', 'rotation_right']]
    if role == 'jump_test':
        labels = [[],['jump_left'], [], ['jump_forward', 'jump_backward'], ['jump_right']]
    elif role == 'kick_test':
        labels = [[],[],[],['regular', 'new_kick'], []]
    else: 
        labels = [[], [], [], ['start', 'start_later'], []]
    if role == 'forward':
        second_pressed_button = 'start'
    else:
        second_pressed_button = motion.push_Button(labels)
    if role == 'dance':
        local = None
    else:
        motion.direction_To_Attack = -initial_coord[2]
        motion.activation()
        local = Local(motion, glob, vision, coord_odometry = initial_coord)
        motion.local = local
        local.coordinate_record(odometry = True)
    print( role, subrole, ' initial_coord = ', initial_coord)
    player = Player(role, second_pressed_button, glob, motion, local)
    player.play_game()

except Exception as e:
    exc_type, exc_value, exc_traceback = sys.exc_info()
    f = open("Error.txt",'w')
    traceback.print_exception(exc_type, exc_value, exc_traceback,
                          limit=None, file=f)
    f.close()
    traceback.print_exception(exc_type, exc_value, exc_traceback,
                          limit=None, file=sys.stdout)
