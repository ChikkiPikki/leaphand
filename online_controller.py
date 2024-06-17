#!/usr/bin/env python3
import mujoco
import mujoco.gl_context
import mujoco.viewer
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import time
from math import pi
import mujoco
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import *

import random
from mujoco_to_real import DisplayNode
from online_receiver import OnlineController
'''
This file contains the API to allow the user
to remotely control a leaphand using the mujoco
interface
'''

def main():
    Client = DisplayNode()
    Receiver = OnlineController()
    while True:
        joint_angles = Receiver.get_command()
        if(joint_angles):
            Client.update_state(np.array(joint_angles[1]))
            print(joint_angles)

if __name__=='__main__':
    main()

