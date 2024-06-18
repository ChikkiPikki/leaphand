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

class MujocoInterface():
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path("/home/tanay/catkin_ws/src/leap_hand/leap_hand_utils/others/example.xml")
        self.d = mujoco.MjData(self.m)
    def viewer(self):
        return mujoco.viewer.launch_passive(self.m, self.d)
    def send_jacobian(self, req):
        pass
'''
The aim of this file is to display the current
configuration of the hand inside mujoco, and to allow 
the manual control of the hand
'''
def main():
    Mujoco = MujocoInterface()
    m = Mujoco.m
    d = Mujoco.d
    with Mujoco.viewer() as viewer:
        start = time.time()
        step = 0
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(m, d)
            with viewer.lock():
                # Get the position of the leap hand
                # Set the joint' position to the position of the leap hand
                # minus pi
                lp = d.qpos
            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time()-step_start)
            step+=1
            if(time_until_next_step>0):
                time.sleep(time_until_next_step)
if __name__=="__main__":
    main()