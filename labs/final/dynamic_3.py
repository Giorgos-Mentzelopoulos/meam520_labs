import sys
import numpy as np
from copy import deepcopy

from math import pi

import rospy
import roslib

# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

# The library you implemented over the course of this semester!
from lib.calculateFK import FK
#from lib.calcJacobian import FK
from lib.solveIK import IK
# from lib.rrt import rrt
# from lib.loadmap import loadmap


####################### Global Variables ###############################

Htag0_rob = np.array([
       [1.0,0.0,0.0,0.5],
       [0.0,1.0,0.0,0.0],
       [0.0,0.0,1.0,0.0],
       [0.0,0.0,0.0,1.0]])

H_red_goal = np.array([
       [1.0,0.0,0.0,0.57],
       [0.0,-1.0,0.0,0.18],
       [0.0,0.0,-1.0,0.26],
       [0.0,0.0,0.0,1.0]])

H_blue_goal = np.array([
        [1.0,0.0,0.0,0.57],
        [0.0,-1.0,0.0,-0.12],
        [0.0,0.0,-1.0,0.26],
        [0.0,0.0,0.0,1.0]])

H_dynamic_red = np.array([
        [1.0,0.0,0.0,0],
        [0.0,-1.0,0.0,0.7],
        [0.0,0.0,-1.0,0.23],
        [0.0,0.0,0.0,1.0]])

H_dynamic_blue = np.array([
        [1.0,0.0,0.0,0],
        [0.0,-1.0,0.0,-(0.70)],
        [0.0,0.0,-1.0,0.23],
        [0.0,0.0,0.0,1.0]])

#### Kostantinos Provided this #####
# H_dynamic_blue = np.array([
#         [0,-0.5,0.866,-0.19],
#         [0.0,-0.866,-0.5,-(0.71)],
#         [1,0.0,-1.0,0.23],
#         [0.0,0.0,0.0,1.0]])

# H_start_red = H_dynamic_red
# H_start_red[2,3] += 0.05
#
# # H_start_blue = H_dynamic_blue
# H_start_blue[2,3] += 0.05

H_start_red = np.array([
        [1.0,0.0,0.0,0],
        [0.0,-1.0,0.0,0.7],
        [0.0,0.0,-1.0,0.27],
        [0.0,0.0,0.0,1.0]])

H_start_blue = np.array([
        [1.0,0.0,0.0,0],
        [0.0,-1.0,0.0,-(0.70)],
        [0.0,0.0,-1.0,0.27],
        [0.0,0.0,0.0,1.0]])

dynamic_seed_red = np.array([-0.85566999, -1.00246097,  1.86687916, -1.49717099,  0.96796238,  1.77792418, 1.72581234])
conf_above_blocks_red = np.array([-1.13061199, -1.08598736,  1.94207577, -1.01762171,  0.96986495,  1.59907308,  1.88032681])


dynamic_seed_blue = np.array([-1.2776221,  0.79439959, -0.25177846, -1.15526736,  0.19084159,  1.92761101, -0.73793715])
conf_above_blocks_blue = np.array([-1.28378389,  0.76875326, -0.25657582, -1.07030855,  0.18300112,  1.81841073, -0.7301028 ])

neutral_pose = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014,  0.02984053,  1.54119353, 0.75344866])

grab_correction_blue = -0.3 #neutral = -0.88 and other way = -1.3

grab_correction_red = 1.5 #neutral = 1.88 and other wat = 2.1

H_goal = [] # keep track of updated H_goal based on stack height & platform color
num_stacked = 0 # keep track of number of stacked blocks to adjust z target


ik = IK()


def grab_dynamic_block():
    #Input: H_target from "get_tag_frame"
    #Output: returns whether robot end effector successfully moved to location of tag and grabs box
    arm.open_gripper()
    print("H_start: ", H_start)
    q_start,success,rollout = ik.inverse(H_start, conf_above_blocks)
    print("Found Starting Configuration: ", success)
    print("Configuration above blocks: ", q_start)
    print("Moving to configuration above blocks")
    q_start[6] = grab_correction  ######## For blue team
    arm.safe_move_to_position(q_start)

    print("Searching for configuration to grab blocks")
    print("H_dynamic: ", H_dynamic)
    q_grab,success,rollout = ik.inverse(H_dynamic, dynamic_seed)
    print("Found Configuration to grab blocks: ", success)
    print("Configuration to grab blocks: ", q_grab)
    print("Moving to configuration to grab blocks")
    q_grab[6] = grab_correction ######## For blue team
    arm.safe_move_to_position(q_grab)
    arm.exec_gripper_cmd(0.02,50)
    gripper_state = arm.get_gripper_state()

    counter = 0
    while sum(gripper_state['force']) < 5 and counter < 1:
        arm.open_gripper()
        arm.exec_gripper_cmd(0.02,50)
        gripper_state = arm.get_gripper_state()
        # print('Gripper Force: ', gripper_state['force'])
        # print('Sum Gripper Force:', sum(gripper_state['force']))
        # print('Counter: ', counter)
        counter += 1
    return success


def take_tag_to_goal(q_current=neutral_pose):
    #Input: Could be specified - otherwise assumed to be the neutral neutral_position
    #output: Places the block that is ALREADY GRASPED to the goal H_platform
    #Note: I need to fix the frame of the goal platform -- see global variables section H_goal
    global num_stacked
    print('H_goal before addition: ',H_goal )
    H_goal[2,3] += 0.05*num_stacked
    print("Searching for Goal Configuration")
    print('Number of Stacked Blocks: ', num_stacked)
    print('H_goal after addition: ',H_goal )
    q,success,rollout = ik.inverse(H_goal, q_current)
    print("take to goal success", success)
    if success:
        num_stacked += 1
        arm.safe_move_to_position(q)
        arm.open_gripper()

def go_to_neutral():
    arm.safe_move_to_position(arm.neutral_position())


if __name__ == "__main__":


    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    arm.safe_move_to_position(arm.neutral_position()) # on your mark!


    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
        H_goal = H_blue_goal
        H_dynamic = H_dynamic_blue
        H_start = H_start_blue
        dynamic_seed = dynamic_seed_blue
        conf_above_blocks = conf_above_blocks_blue
        grab_correction = grab_correction_blue



    else:
        print("**  RED TEAM  **")
        H_goal = H_red_goal
        H_dynamic = H_dynamic_red
        H_start = H_start_red
        dynamic_seed = dynamic_seed_red
        conf_above_blocks = conf_above_blocks_red
        grab_correction = grab_correction_red




    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!


    # STUDENT CODE HERE
    # print("H_start: ", H_start)
    # print("H_dynamic: ", H_dynamic)

    while True:
        successfully_grabbed = grab_dynamic_block()
        go_to_neutral() # Takes robot to neutral
        take_tag_to_goal() # Attempts to take box to the goal platform
        go_to_neutral() # Takes robot to neutral
