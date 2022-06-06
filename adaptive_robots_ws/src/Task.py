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
        self.limb_name = limb
        self.hover_dist = hover_dist
        self.limb = baxter_interface.Limb(limb)
        self.gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()

    '''---------------------pick and place----------------'''
    def pick(self, pose):
        self.open_gripper()
        self.approach()
        self.move_servo(pose)
        self.close_gripper()
        self.retract()

    def place(self, pose, key):
        self.approach()

        if key == "F":
            self.open_gripper()
            self.move_servo(pose)
        else:
            self.move_servo(pose)
            self.open_gripper()
        self.retract()
    '''-------------------------Gripper-------------------------'''

    def open_gripper(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def close_gripper(self):
        self.gripper.close()
        rospy.sleep(1.0)

    def calibrate_gripper(self):
        self.gripper.callibrate()
    '''------------------------Movement-------------------------'''

    def gaurded_move(self, joint_angles):
        if joint_angles:
            self.limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles Given")

    def move_servo(self, pose):
        joint_angles = self.ik_request(pose)
        self.gaurded_move(joint_angles)

    def move_to_start(self, start_angles=None):
        rospy.logerr("Moving to Start")
        if not start_angles:
            start_angles = dict(zip(self.joint_names, [0]*7))
        self.gaurded_move(start_angles)
        self.open_gripper()
        rospy.sleep(1.0)
    '''--------------------Helpers----------------------------'''

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
            }.get(resp_seeds[0], 'None')
            if self._verbose:
                limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def approach(self, pose):
        approach = copy(deepcopy(pose))
        approach.position.z = approach.position.z + self.hover_dist
        joint_angles = self.ik_request(approach)
        self.gaurded_move(joint_angles)

    def retract(self):
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)



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
    idx = 0
    while not rospy.is_shutdown():
        getKey()
        rospy.logerr("Picking...")
        tsk1.pick(block_poses[idx])
        if key:
            rospy.logerr("Failure")
        else:
            rospy.logerr("Placing")
        tsk1.place(block_poses[idx], key)
        idx = (idx + 1) % len(block_poses)
    return 0

if __name__ == '__main__':
    sys.exit(main())

