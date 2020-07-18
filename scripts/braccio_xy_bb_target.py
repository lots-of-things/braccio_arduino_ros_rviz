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

l = 1/np.cos(0.26)

def get_targets(x,y):
    r, phi = cart2pol(x,y)
    if r > l*np.cos(0.26) or r < l*np.cos(0.77):
        print '++++++ Not in Domain ++++++'
        print 'r = ' + str(r)
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
    self.R = np.array([[1,0],[0,1]])
    self.scale = 1
    self.x0 = 0
    self.y0 = 0

  def callback(self, ros_data):
    self.bounding_box = ros_data.data

  def transform(self, x1, y1):
    return self.R.dot([x1-self.x0, y1-self.y0]/self.scale)

  def transform_bb(self):
    x, y = self.bounding_box_center()
    return self.transform(x,y)

  def bounding_box_center(self):
    x = (self.bounding_box[1]+self.bounding_box[3])/2
    y = (self.bounding_box[2]+self.bounding_box[4])/2
    return x, y

  def calibrate(self):
    print 'place apple at base, when ready press enter'
    raw_input()

    x0s = []
    y0s = []
    for i in range(10):
      x, y = self.bounding_box_center()
      x0s.append(x)
      y0s.append(y)
      time.sleep(1)

    x0 = np.mean(x0s)
    y0 = np.mean(y0s)
    print 'place apple at furthest extent centered, when ready press enter'
    raw_input()

    x1s = []
    y1s = []
    for i in range(10):
      x, y = self.bounding_box_center()
      x1s.append(x)
      y1s.append(y)
      time.sleep(1)
    x1 = np.mean(x1s)
    y1 = np.mean(y1s)

    self.x0 = x0
    self.y0 = y0

    ang = np.arctan2(1,0) - np.arctan2(y1-y0,x1-x0)
    self.R = np.array([[np.cos(ang),-np.sin(ang)],[np.sin(ang),np.cos(ang)]])
    v = np.array([x1-self.x0,y1-self.y0])
    self.scale = np.sqrt(v.dot(v.T))
    print 'calibration done.'
    print self.transform_bb()

  def go_to(self, x, y):

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

  def go_to_joint_state(self):

    print 'pos x?'
    tst = raw_input()
    if tst!='':
        x = float(tst)
    print 'pos y?'
    tst = raw_input()
    if tst!='':
        y = float(tst)

    self.go_to(x, y)

  def go_to_target_joint_state(self):
    v = self.transform_bb()
    self.go_to(v[0], v[1])

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
    x, y = self.bounding_box_center()
    print x, y
    print self.transform(x, y)

def main():
  try:
    bb_targetter = BraccioXYBBTargetInterface()

    while True:
        print "============ instructions: p=print, c=calibrate, t=target, m=manual, h=home, u=up, q=quit"
        inp = raw_input()
        if inp=='q':
            break
        if inp=='p':
            bb_targetter.print_pose()
        if inp=='c':
            bb_targetter.calibrate()
        if inp=='t':
            bb_targetter.go_to_target_joint_state()
        if inp=='m':
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
