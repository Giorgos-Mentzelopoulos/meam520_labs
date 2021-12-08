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
from lib.rrt import rrt
from lib.loadmap import loadmap



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

    else:
        print("**  RED TEAM  **")

    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE

    ik = IK()


    ####################### Global Variables ###############################
    Htag0_rob = np.array([
       [1.0,0.0,0.0,0.5],
       [0.0,1.0,0.0,0.0],
       [0.0,0.0,1.0,0.0],
       [0.0,0.0,0.0,1.0]])

    H_goal = np.array([
       [1.0,0.0,0.0,0.3],
       [0.0,-1.0,0.0,0.3],
       [0.0,0.0,-1.0,0.23],
       [0.0,0.0,0.0,1.0]])


   #################### Useful Functions #####################################
    def get_tag_frame(tag):
        #Input: tag = string - specifies which tag to grab
        #Note: specified tag must be on table for this to work
        #Note: has not been tested for dynamic blocks
        #Output: Returns the frame of the specified tag (input) relative to the robot
        #Feed the output to "grab_tag_frame"
        for (name,pose) in detector.get_detections():
            if name == "tag0":
                Hcam_tag0 = pose
            if name == tag:
                H = pose

        Htag_rob = np.linalg.inv(H)@Hcam_tag0@Htag0_rob
        Hrob_tag = np.linalg.inv(Htag_rob)

        H_target = np.identity(4)

        H_target[0,3] = Hrob_tag[0,3]
        H_target[1,3] = Hrob_tag[1,3]
        H_target[2,3] = Hrob_tag[2,3]
        H_target[1,1] = -1
        H_target[2,2] = -1

        return H_target

    def grab_tag_frame(H_target, q_current=arm.neutral_position()):
        #Input: H_target from "get_tag_frame"
        #Output: Takes robot end effector to location of tag and grabs box
        arm.open_gripper()
        q,success,rollout = ik.inverse(H_target, q_current)
        arm.safe_move_to_position(q)
        arm.exec_gripper_cmd(0.05,15)

    def take_tag_to_goal(q_current=arm.neutral_position()):
        #Input: Could be specified - otherwise assumed to be the neutral neutral_position
        #output: Places the block that is ALREADY GRASPED to the goal H_platform
        #Note: I need to fix the frame of the goal platform -- see global variables section H_goal
        q,success,rollout = ik.inverse(H_goal, q_current)
        arm.safe_move_to_position(q)
        arm.open_gripper()

    def go_to_neutral():
        arm.safe_move_to_position(arm.neutral_position())


    ######################## Main Function ##########################

    #Specify the "tag" you want the robot to grab
    tag = 'tag6'
    H_target = get_tag_frame(tag) #extracts homologous transformation for that tag
    grab_tag_frame(H_target) # Takes robot to tag and grabs box
    go_to_neutral() # Takes robot to neutral
    take_tag_to_goal() # Attempts to take box to the goal platform
    go_to_neutral() # Takes robot back to neutral position







    ######################## Neglect below this point #####################

    #Detect some tags...
    # for (name, pose) in detector.get_detections():
    #      print(name,'\n',pose)

    # arm.open_gripper()
    # q,success,rollout = ik.inverse(H_target, arm.neutral_position())
    # arm.safe_move_to_position(q)
    # arm.exec_gripper_cmd(0.05,10)
    # arm.safe_move_to_position(arm.neutral_position())

    # H_platform = np.array([
    #    [1.0,0.0,0.0,0.894],
    #    [0.0,-1.0,0.0,0.562],
    #    [0.0,0.0,-1.0,-0.24],
    #    [0.0,0.0,0.0,1.0]])
    #
    # q_platform,_,_ = ik.inverse(H_platform, arm.neutral_position())
    # arm.safe_move_to_position(q_platform)
    # arm.open_gripper()
    # arm.safe_move_to_position(arm.neutral_position())






















    # END STUDENT CODE
