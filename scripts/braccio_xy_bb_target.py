#!/usr/bin/env python

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Int16MultiArray

from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL
import numpy as np


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

l = 1

def get_targets(x,y):
    r, phi = cart2pol(x,y)
    if r > l*np.cos(0.26) or r < l*np.cos(0.77):
        print '++++++ Not in Domain ++++++'
        return None
    theta_shoulder = np.arccos(r/l)
    theta_wrist = theta_shoulder + np.pi/2
    theta_elbow = np.pi/2 - 2*theta_shoulder
    return [phi, theta_shoulder, theta_elbow, theta_wrist]


class BraccioXYBBTargetInterface(object):
  """BraccioXYBBTargetInterface"""
  def __init__(self):
    super(BraccioXYBBTargetInterface, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('braccio_xy_bb_target', anonymous=True)

    group_name = "braccio_arm"
    self.move_group = moveit_commander.MoveGroupCommander(group_name)

    self.bounding_box = Int16MultiArray()
    self.subscriber = rospy.Subscriber("bounding_box",  Int16MultiArray, self.callback, queue_size=1)

  def callback(self, ros_data):
    self.bounding_box = ros_data

  def go_to_joint_state(self):

    print 'pos x?'
    tst = raw_input()
    if tst!='':
        x = float(tst)
    print 'pos y?'
    tst = raw_input()
    if tst!='':
        y = float(tst)

    joint_targets = get_targets(x,y)

    print joint_targets
    if joint_targets:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = joint_targets[0]
        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()

        joint_goal[1] = joint_targets[1]
        joint_goal[2] = joint_targets[2]
        joint_goal[3] = joint_targets[3]

        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()

        current_joints = self.move_group.get_current_joint_values()

  def go_to_home_state(self):

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[1] = 1.15
    joint_goal[2] = 0.13
    joint_goal[3] = 2.29

    self.move_group.go(joint_goal, wait=True)

    self.move_group.stop()

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 3.14

    self.move_group.go(joint_goal, wait=True)

    self.move_group.stop()

  def go_to_up_state(self):

    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = 1.5708
    joint_goal[1] = 1.5708
    joint_goal[2] = 1.5708
    joint_goal[3] = 1.5708

    self.move_group.go(joint_goal, wait=True)

    self.move_group.stop()


  def print_pose(self):
    print self.move_group.get_current_pose().pose
    print self.bounding_box

def main():
  try:
    bb_targetter = BraccioXYBBTargetInterface()

    while True:
        print "============ instructions: p=print, c=control, h=home, u=up, q=quit"
        inp = raw_input()
        if inp=='q':
            break
        if inp=='p':
            bb_targetter.print_pose()
        if inp=='c':
            bb_targetter.go_to_joint_state()
        if inp=='h':
            bb_targetter.go_to_home_state()
        if inp=='u':
            bb_targetter.go_to_up_state()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
