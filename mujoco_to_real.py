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

'''
The aim of this file is to display the current
configuration of the hand inside mujoco, and to allow 
the manual control of the hand
'''

class DisplayNode():
    def __init__(self):
        rospy.init_node("mujoco_to_real")
        rospy.wait_for_service("/leap_position")
        self.pub_hand = rospy.Publisher("/leaphand_node/cmd_leap", JointState, queue_size=1)        
    def update_state(self, position):
        state = JointState()
        state.position = position
        self.pub_hand.publish(state)
        
class MujocoInterface():
    def __init__(self):
        self.jacobian = rospy.Service("/leap_jacobian", mujoco_info, self.send_jacobian)
        self.m = mujoco.MjModel.from_xml_path("/home/tanay/catkin_ws/src/leap_hand/leap_hand_utils/leap_hand/robot.urdf")
        self.d = mujoco.MjData(self.m)
    def viewer(self):
        return mujoco.viewer.launch_passive(self.m, self.d)
    def send_jacobian(self, req):
        pass
        

        
        
def main():
    Mujoco = MujocoInterface()
    Client = DisplayNode()
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
                print(d.friction )
                d.qpos = []
                if(step%5 == 0):
                    Client.update_state([x+3.14 for x in lp])
            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time()-step_start)
            step+=1
            if(time_until_next_step>0):
                time.sleep(time_until_next_step)
if __name__=="__main__":
    main()
    