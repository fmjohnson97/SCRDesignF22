"""
on input of directional key, use left and right to turn the robot. use front and back to move
"""

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist, PoseStamped
import copy
import cv2
import time
import numpy as np
import transformations as tf
class SimplePatrol:
  def __init__(self):
    # self.pub = rospy.Publisher('/locobot/cmd_vel', Twist)
    self.pub = rospy.Publisher('/locobot/move_base_simple/goal', PoseStamped)
    self.linear_v = 0.08
    self.ang_v = 0.08
    rospy.sleep(1.0)
  def control(self, x, theta):
    """
    low-level control of the robot to move in the x axis by x, and rotate by theta
    """
    msg = PoseStamped()
    msg.header.frame_id = 'base_link'
    msg.pose.position.x = x
    rot_mat = tf.rotation_matrix(theta, [0,0,1])
    qw, qx, qy, qz = tf.quaternion_from_matrix(rot_mat)
    msg.pose.orientation.w = qw
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz

    self.pub.publish(msg)
    print('msg: ')
    print(msg)
    # rospy.sleep(1.0)
    input('next...')
  def step(self, rel_pose):
    """
    go from my current position to the relative pose
    rel_pose: [x,y,theta]
    - x, y are the relative positions from my current
    - theta is the relative orientation from my current
    Step:
    1. rotate to face [x,y] orientation: theta'
    2. step [x,y] toward the goal
    3. rotate by theta - theta' to achieve the goal orientation
    """
    # Step 1: rotate to face the goal
    goal_x = rel_pose[0]
    goal_y = rel_pose[1]
    theta1 = np.arctan2(goal_y, goal_x)
    print('thetat1: ', theta1)
    # self.control(0, theta1)

    # Step 2: move forward
    goal_x = rel_pose[0]
    goal_y = rel_pose[1]
    dis = np.sqrt(goal_x**2+goal_y**2)
    print('thetat1: ', theta1)
    self.control(dis, 0)

    # Step 3: rotate again
    self.control(0, rel_pose[2]-theta1)

if __name__ == "__main__":
  rospy.init_node('patrol', anonymous=True)
  controller = SimplePatrol()

  ang = -45
  waypoint = [0.5, 0, ang*np.pi/180]
  controller.step(waypoint)