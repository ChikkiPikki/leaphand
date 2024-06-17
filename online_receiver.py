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

import asyncio
import json
import socketio
from mujoco_to_real import MujocoInterface
class OnlineController:
    def __init__(self):
        '''
        Initialize the ros and the websocket client
        '''
        self.sio = socketio.SimpleClient()
        self.sio.connect("http://localhost:4000")
        
    def get_command(self):
        return self.sio.receive()
    def publish_command(self, angles):
        '''
        Publish the joint angles over the
        node js interface
        '''
        self.sio.emit("leap_angles", angles)
    def set_mode():
        '''
        Set the mode to control or observation.
        If the mode is control, the Leaphand will be
        controlled by the received commands.
        If the mode is observation, the current
        position of the leaphand will be broadcasted
        '''
        pass
    
def main():
    Client = OnlineController()
    Client.publish_command([1,1,1,1,1])
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
                if(step%5 == 0):
                    Client.publish_command([x+3.14 for x in lp])
            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time()-step_start)
            step+=1
            if(time_until_next_step>0):
                time.sleep(time_until_next_step)

if __name__=='__main__':
    main()