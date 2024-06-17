#!/usr/bin/env python3
import os
import sys
import time
import numpy as np
import rospy

from sensor_msgs.msg import JointState

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import *

# This is example code, it reads the position from LEAP Hand and commands it
# Be sure to query the services only when you need it
# This way you don't clog up the communication lines and you get the newest data possible
class Telekinesis:
    def __init__(self):        
        rospy.wait_for_service('/leap_position')
        self.leap_position = rospy.ServiceProxy('/leap_position', leap_position)
        #self.leap_velocity = rospy.ServiceProxy('/leap_velocity', leap_velocity)
        #self.leap_effort = rospy.ServiceProxy('/leap_effort', leap_effort)
        self.pub_hand = rospy.Publisher("/leaphand_node/cmd_leap", JointState, queue_size = 3) 
        r = rospy.Rate(10)   #30hz rate
        while not rospy.is_shutdown():
            r.sleep()
            ###only run these services if you need them.
            curr_pos = np.array(self.leap_position().position)
            #curr_vel = np.array(self.leap_velocity().velocity)
            #curr_eff = np.array(self.leap_effort().effort)
            print(curr_pos)
            ###This is a fresh position, now do some policy stuff here etc.
            
            #Set the position of the hand when you're done
            # stater = JointState()
            # stater.position = np.zeros(16)
            state = JointState()
            state.position = [3.87023354, 3.1461947,  5.0053792,  4.46234989, 3.65394235, 3.16613626,
                5.03759289, 4.36417532, 3.81961226, 3.2351656,  5.00998116, 4.34269953,
                3.51434994, 4.40866089, 4.21844721, 4.84891319]
            # self.pub_hand.publish(state)  ##choose the right embodiment here
            print("hi")
if __name__ == "__main__":
    rospy.init_node("ros_example")
    telekinesis_node = Telekinesis()
