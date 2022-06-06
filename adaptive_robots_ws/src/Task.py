#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import rospy
import rospkg
from geometry_msg.msg import (
    PoseStamped, Pose, Point, Quaternion
)
from std_msgs.msg import (
    Header, Empty
)
from baxter_core_msgs.srv import (
    SolvePositionIK, SolvePositionIKRequest
)
import baxter_interface

'''-----------------------------------------------------------------'''


'''
Get command prompt string input from different machines
'''
def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class Task1(object):

    def __init__(self, limb, hover_dist=0.5):
        self.limb = limb
        self.hover_dist = hover_dist
        '''---------------------pick and place----------------'''
    def pick(self, pose):
        pass

    def place(self, pose, key):
        if key == "F":
            pass
        else:
            pass
        pass
    '''-------------------------Gripper-------------------------'''

    def open_gripper(self):
        pass

    def close_gripper(self):
        pass

    def calibrate_gripper(self):
        pass
    '''------------------------Movement-------------------------'''

    def gaurded_move(self, pose):
        pass

    def move_servo(self, pose):
        pass

    def move_to_start(self):
        pass
    '''--------------------Helpers----------------------------'''

    def ik_request(self, pose):
        pass

    def approach(self):
        pass

    def retract(self):
        pass



def main():
    rospy.init_node("task1")
    rospy.Subscriber('', String, callback)

    limb = 'left'
    hover_dist = 0.15  # in meters
    tsk1 = Task1(limb, hover_dist)

    # Starting Joint angles for left arm
    starting_joint_angles = {
        'right_w0': 0.6699952259595108,
        'right_w1': 1.030009435085784,
        'right_w2': -0.4999997247485215,
        'right_e0': -1.189968899785275,
        'right_e1': 1.9400238130755056,
        'right_s0': -0.08000397926829805,
        'right_s1': -0.9999781166910306}

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
        x=-0.0249590815779,
        y=0.999649402929,
        z=0.00737916180073,
        w=0.00486450832011)

    '''------------Replace with things from perception nodes------------'''
    # create a list of poses for the robot to pick and place from
    # first pose will be picked from and second placed
    block_poses = list()
    block_poses.append(Pose(
        position=Point(x=-1, y=0.15, z=0),
        orientation=overhead_orientation))
    block_poses.append(Pose(
        position=Point(x=0, y=0.5, z=0),
        orientation=overhead_orientation))

    # This section handles picking and placing using the pose list
    # the user can update on the terminal and have a place
    # command fail
    indx = 0
    while not rospy.is_shutdown():
        getKey()
        print("\n Picking...")
        tsk1.pick(block_poses[idx])
        if key:
            print("\n Placing Failure")
        else:
            print("\n Placing")
        tsk1.place(block_poses[idx], key)
        idx = (idx + 1) % len(block_poses)
    return 0

if __name__ == '__main__':
    sys.exit(main())

