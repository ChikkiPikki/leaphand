#!/usr/bin/env python3
import numpy as np
import rospy

from sensor_msgs.msg import JointState

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import *

import random

'''
The aim of this file is to display how we are
controlling the hand in simulation as well as
reality. This is demonstrated by playing a 
game of rock-paper-scissors with the hand
'''

class ControlNode():
    def __init__(self):
        rospy.wait_for_service("/leap_position")
        self.leap_position = rospy.ServiceProxy('/leap_position', leap_position)
        self.pub_hand = rospy.Publisher("/leaphand_node/cmd_leap", JointState, queue_size=1)
    def _pub_state(self, position):
        state = JointState()
        state.position = position
        self.pub_hand.publish(state)
            
    def go_to_rock(self):
        self._pub_state([3.87023354, 3.1461947,  5.0053792,  4.46234989, 3.65394235, 3.16613626,
                5.03759289, 4.36417532, 3.81961226, 3.2351656,  5.00998116, 4.34269953,
                3.51434994, 4.40866089, 4.21844721, 4.84891319])
        print("rock")
    def go_to_paper(self):
        self._pub_state([2.81638885, 2.69520426 ,3.13852477, 3.13852477 ,2.96058297, 3.18761206,
 3.11551499, 3.14312673, 2.90842748 ,3.86563158, 3.035748 , 3.15386438,
 2.80565095, 4.47768974 ,3.65854406 ,2.82866049])
        print("paper")
    def go_to_scissors(self):
        self._pub_state(
            [2.85627222, 2.69827223, 3.14005876, 3.14312673, 3.08943725, 3.19681597,
 3.11704898, 3.14312673, 4.84431124, 3.36401987, 4.78908825, 3.14159274,
 3.91011691, 4.02363157, 4.76914644, 3.20601988]
        )
        print("scissors")
    def go_to_home(self):
        self._pub_state(
            [2.76883531, 3.33180618, 3.13852477, 3.1461947,  2.94677711, 3.65087438,
 3.11551499, 3.14312673, 2.94677711, 3.99295211, 3.1830101,  2.77497125,
 3.28118491, 4.64029169, 3.3302722,  2.73508763]
        )
        rospy.sleep(0.7)
        self._pub_state(
            [2.78570914, 2.19512653, 2.65685463, 3.53275776, 2.88695192, 2.75349545,
 3.08790326, 2.79644704, 2.83479643, 2.96671891, 2.6599226, 3.2612431,
 3.00353432, 3.33180618, 3.80427241, 3.07563138]
        )
        rospy.sleep(0.7)
def main():
    LEAP_Hand_Control = ControlNode()
    LEAP_Hand_Control.go_to_home()
    choice = [LEAP_Hand_Control.go_to_paper, 
              LEAP_Hand_Control.go_to_rock, 
              LEAP_Hand_Control.go_to_scissors]
    choice[random.randint(0,2)]()

if __name__=='__main__':
    rospy.init_node("leaphand_node_2")
    main()
