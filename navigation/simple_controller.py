"""
A simple PID-like controller to control the robot to navigate to a target location (dx,dy,dtheta),
which is specified relative to the start pose.
Obtain the odometry information of the robot, and track it until the pose is realized.
"""


import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import copy
import cv2
import time
import numpy as np
import transformations as tf

class SimpleController:
  def __init__(self):
    self.pub = rospy.Publisher('/locobot/cmd_vel', Twist)
    self.linear_v = 0.08  # this is the max limit of the velocity value
    self.ang_v = 2*np.pi/180  # positive is counter-clockwise, negative is clockwise
    self.x_threshold = 1e-3
    self.theta_threshold = 1 * np.pi / 180
    rospy.sleep(1.0)
    """
    TODO: the current rotation seems to be a bit off. When we ask it to go 90 degrees, it dosen't really go 90 degrees.
    Need to experiment and debug the rotation controller
    """

  def get_odom(self):
    odom = rospy.wait_for_message('/locobot/odom', Odometry)
    pose = odom.pose.pose
    pose_3d = tf.transformations.quaternion_matrix([pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z])
    pose_3d[0,3] = pose.position.x
    pose_3d[1,3] = pose.position.y
    pose_3d[2,3] = pose.position.z
    twist = odom.twist.twist
    twist = [twist.linear.x, twist.linear.y, twist.angular.z]
    return pose_3d, twist


  def pose_3d_to_2d(self, pose_3d_mat):
    angles = tf.transformations.euler_from_matrix(pose_3d_mat, axes='sxyz')
    return [pose_3d_mat[0,3], pose_3d_mat[1,3], angles[2]]

  def control_x(self, x):
    """
    low-level control of the robot to move in the facing axis by x (positive: forward, negative: backward)
    Added on Oct 21: stablize the robot to avoid drifting, by using the odometry information to get the relative
    rotation, and use PID-like controller to give velocity command to counter the drifting effect.
    TODO: The robot may still have some positional different in the y axis. A better controller might be to
    track the robot back in the y axis too.
    """
    # obtain the start pose through odometry
    pose, twist = self.get_odom()
    # obtain the start frame of the robot (the facing frame of the robot) in the odom frame
    # we need to use this to compute the relative pose of the robot later frame in this one
    # to be able to compute the positional difference and angular ones
    start_pose = np.array(pose)
    # x0 = pose[0]

    msg = Twist()

    rate = rospy.Rate(40)
    while True:
      pose, twist = self.get_odom()  # get current odometry
      # x1 = pose[0]
      print('tracking position...')
      # obtain the relative pose
      # del_pose = pose.dot(np.linalg.inv(start_pose))
      cur_pose_in_start = np.linalg.inv(start_pose).dot(pose)
      # project the relative pose to 2d
      dx, dy, dth = self.pose_3d_to_2d(cur_pose_in_start)
      diff = x-dx

      ang_diff = 0-dth  # we want to stablizie the robot when moving forward. So the target angular value is 0

      print('current pose in start:', cur_pose_in_start)
      print('diff: ', diff)
      print('ang_diff: ', ang_diff)

      if np.abs(diff) < self.x_threshold:
        break
      abs_value = np.abs(self.linear_v * diff*10)
      abs_value = min(abs_value, self.linear_v)
      msg.linear.x = abs_value * np.sign(diff)
      print('position velocity: ', abs_value * np.sign(diff))

      # for stabalizing the drift
      abs_value = np.abs(self.ang_v * ang_diff*10)
      abs_value = min(abs_value, self.ang_v)
      msg.angular.z = abs_value * np.sign(ang_diff)

      self.pub.publish(msg)
      rate.sleep()

  def control_theta(self, theta):
    """
    low-level control of the robot to rotate by theta. Relative to the start pose of the robot
    """
    # obtain the start pose through odometry
    pose, twist = self.get_odom()
    # convert the pose in the odom frame to the robot's current facing frame (using the current orientation)
    start_pose = np.array(pose)
    # x0 = pose[0]

    msg = Twist()

    rate = rospy.Rate(40)
    while True:
      pose, twist = self.get_odom()
      # x1 = pose[0]
      print('tracking rotation...')
      # obtain the relative pose
      # del_pose = pose.dot(np.linalg.inv(start_pose))
      cur_pose_in_start = np.linalg.inv(start_pose).dot(pose)
      # get the angle
      dx, dy, dth = self.pose_3d_to_2d(cur_pose_in_start)
      diff = theta-dth
      # diff = x-(x1-x0)
      print('current pose in start:', cur_pose_in_start)
      print('diff: ', diff)


      if np.abs(diff) < self.theta_threshold:
        break
      abs_value = np.abs(self.ang_v * diff*10)
      abs_value = min(abs_value, self.ang_v)
      msg.angular.z = abs_value * np.sign(diff)
      print('angle velocity: ', abs_value * np.sign(diff))

      self.pub.publish(msg)
      rate.sleep()

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
    self.control_theta(theta1)
    rospy.sleep(2)
    # Step 2: move forward
    goal_x = rel_pose[0]
    goal_y = rel_pose[1]
    dis = np.sqrt(goal_x**2+goal_y**2)
    print('moving forward by: ', dis)
    self.control_x(dis)
    rospy.sleep(2)

    # Step 3: rotate again
    self.control_theta(rel_pose[2]-theta1)

  def global_waypoint_track(self, waypoints, base_frame=None):
    """
    specify waypoints in the specified base frame (the robot's current pose if None)
    The base frame could be global, so we need to convert that into relative poses to be able
    to use the control functions for tracking.
    TODO
    """
    pass

if __name__ == "__main__":
  rospy.init_node('patrol', anonymous=True)
  controller = SimpleController()

  # * position unit testing *
  # controller.control_x(0.1)

  # * rotation unit testing *
  # controller.control_theta(-10*np.pi/180)

  # * waypoint tracking *
  waypoints = [[1.27, 0.0, 90*np.pi/180],
               [0.8,0, 90*np.pi/180],
               [1.27, 0.0, 90*np.pi/180],
               [0.8, 0.0, 90*np.pi/180]]
  for waypoint in waypoints:
    print('waypoint: ', waypoint)
    controller.step(waypoint)

