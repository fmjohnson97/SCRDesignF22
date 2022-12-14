"""
A motion planner using MoveIt. Low-level motion-planning functions
"""

"""
an interface for motion planning queries
"""
from moveit_commander import move_group
import rospy
import moveit_commander
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from moveit_msgs.msg import PositionIKRequest
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs

import sensor_msgs.point_cloud2 as pcl2
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject

from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle

import std_msgs
from geometry_msgs.msg import PoseStamped, Point
import transformations as tf
from std_srvs.srv import Empty

from moveit_msgs.msg import RobotState, DisplayRobotState
from sensor_msgs.msg import JointState
import sys


from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle

import gc

import pickle

class MotionPlanner():
    def __init__(self, commander_args=[]):
        # set up the scene
        self.PLANNING_NS = '/locobot/'
        moveit_commander.roscpp_initialize(commander_args)
        self.robot_commander = moveit_commander.RobotCommander("%srobot_description" % self.PLANNING_NS, self.PLANNING_NS)
        self.group_arm_name = "interbotix_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_arm_name, "%srobot_description" % self.PLANNING_NS, self.PLANNING_NS)
        self.scene_interface = moveit_commander.PlanningSceneInterface(ns=self.PLANNING_NS)

        self.scene_interface.remove_world_object()
        self.scene_interface.remove_world_object('suction_object')
        rospy.sleep(1.0)
        # self.clear_octomap()
        self.pcd_topic = '/perception/points'
        self.pcd_pub = rospy.Publisher(self.pcd_topic, PointCloud2, queue_size=3, latch=True)

        self.co_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=3, latch=True)

        self.rs_pub = rospy.Publisher('/display_robot_state', DisplayRobotState, queue_size=3, latch=True)
        self.joint_names = self.move_group.get_active_joints()
        print('joint names: ', self.joint_names)

    def save_home_joints(self, fname='home_position'):
        state = self.move_group.get_current_state()
        # extract the joint state
        joint_state = state.joint_state
        f = open(fname+'.pkl', 'wb')
        pickle.dump(joint_state, f)

    def load_home_joints(self):
        f = open('home_position.pkl', 'rb')
        joint_state = pickle.load(f)
        return joint_state

    def load_joints(self, fname):
        f = open(fname+'.pkl', 'rb')
        joint_state = pickle.load(f)
        return joint_state

    def get_state_validity(self, state):
        group_name = self.group_arm_name
        rospy.wait_for_service('/check_state_validity')
        sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = state
        gsvr.group_name = group_name
        result = sv_srv.call(gsvr)
        return result

    def clear_octomap(self):
        # update octomap by clearing existing ones
        rospy.loginfo("calling clear_octomap...")
        rospy.wait_for_service('clear_octomap')
        # generate message
        try:
            ros_srv = rospy.ServiceProxy('clear_octomap', Empty)
            resp1 = ros_srv()

            del ros_srv
            del resp1

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            sys.exit(1)

        gc.collect()
        rospy.sleep(.5)

    def wait(self, time):
        rospy.sleep(time)


    def extract_plan_to_joint_list(self, plan):
        """
        given a motion plan generated by MoveIt!, extract the list of joints

        (success flag : boolean, trajectory message : RobotTrajectory,
         planning time : float, error code : MoveitErrorCodes)
        """
        traj = plan[1]
        traj = traj.joint_trajectory
        joint_names = traj.joint_names
        points = traj.points
        positions = []
        # velocities = []
        # accelerations = []
        time_from_starts = []
        for point in points:
            positions.append(point.positions)
            time_from_starts.append(point.time_from_start)
        if plan[0]:
            return joint_names, positions, time_from_starts
        else:
            return joint_names, [], []

    def tf_to_pose_msg(self, pose):
      msg = Pose()
      msg.position.x = pose[0,3]
      msg.position.y = pose[1,3]
      msg.position.z = pose[2,3]

      qw,qx,qy,qz = tf.quaternion_from_matrix(pose)
      msg.orientation.w = qw
      msg.orientation.x = qx
      msg.orientation.y = qy
      msg.orientation.z = qz
      return msg

    def pose_msg_to_tf(self, msg: PoseStamped):
      pose = tf.quaternion_matrix([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
      pose[0,3] = msg.pose.position.x
      pose[1,3] = msg.pose.position.y
      pose[2,3] = msg.pose.position.z
      return pose


    def pose_motion_plan(self, start_state: JointState, goal_pose, attached_acos):
      """
      plan from start to goal. The start could be chained with previous motion plan
      """
      moveit_robot_state = RobotState()
      moveit_robot_state.joint_state = start_state
      moveit_robot_state.attached_collision_objects = attached_acos
      moveit_robot_state.is_diff = True


      self.move_group.set_planner_id('BiTRRT')
      self.move_group.set_start_state(moveit_robot_state)

      goal_msg = self.tf_to_pose_msg(goal_pose)
      self.move_group.set_pose_target(goal_msg)

      self.move_group.set_planning_time(14)
      self.move_group.set_num_planning_attempts(5)
      self.move_group.allow_replanning(False)


      plan = self.move_group.plan()  # returned value: tuple (flag, RobotTrajectory, planning_time, error_code)
      return plan

    def joint_value_to_list(self, state: JointState):
        """
        get the valid joint names, and then extract those from the joint state
        """
        # first convert to a dict
        joint_dict = {}
        for i in range(len(state.position)):
            joint_dict[state.name[i]] = state.position[i]
        joint_list = []
        for i in range(len(self.joint_names)):
            joint_list.append(joint_dict[self.joint_names[i]])
        return joint_list
    def joint_motion_plan(self, start_state: JointState, joint_state: JointState, attached_acos):
      """
      plan from start to goal. The start could be chained with previous motion plan
      """
      moveit_robot_state = RobotState()
      moveit_robot_state.joint_state = start_state
      moveit_robot_state.attached_collision_objects = attached_acos
      moveit_robot_state.is_diff = True


      self.move_group.set_planner_id('BiTRRT')
      self.move_group.set_start_state(moveit_robot_state)
      print(joint_state.name)
      print(joint_state.position)
      joint_list = self.joint_value_to_list(joint_state)
      self.move_group.set_joint_value_target(joint_list)

      self.move_group.set_planning_time(14)
      self.move_group.set_num_planning_attempts(5)
      self.move_group.allow_replanning(False)


      plan = self.move_group.plan()  # returned value: tuple (flag, RobotTrajectory, planning_time, error_code)
      return plan





    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def display_robot_state(self, state):
        msg = DisplayRobotState()
        msg.state = state
        self.rs_pub.publish(msg)
        print('after publishing...')
        rospy.sleep(1.0)

    def get_state_validity(self, state):
        group_name = self.group_arm_name
        rospy.wait_for_service('/check_state_validity')
        sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = state
        gsvr.group_name = group_name
        result = sv_srv.call(gsvr)
        return result

    def collision_check(self, state, model_name):
        res = self.get_state_validity(state)
        if res.valid:
            return True
        else:
            return False

    def get_ik(self, link_pose, link_name):
        # use the service "compute_ik" to compute the inverse kinematics for the arm
        # http://docs.ros.org/en/api/moveit_msgs/html/srv/GetPositionIK.html
        # TODO: handling the solution and parse it to the correct format for use
        print('get_ik...')
        print('link_pose: ')
        print(link_pose)
        qw, qx,qy, qz = tf.transformations.quaternion_from_matrix(link_pose)
        x,y,z = link_pose[:3,3]
        print('loginfo...')
        rospy.loginfo("calling compute_ik...")
        print('waiting for service...')
        rospy.wait_for_service('/locobot/compute_ik')
        print('after.')
        # generate message
        try:
            ros_srv = rospy.ServiceProxy('/locobot/compute_ik', GetPositionIK)
            req = GetPositionIKRequest()
            msg = PositionIKRequest()
            msg.group_name = 'interbotix_arm'
            msg.robot_state = self.move_group.get_current_state()
            # msg.avoid_collisions = False
            msg.ik_link_name = link_name
            pose = PoseStamped()
            pose.header.frame_id = 'locobot/base_link'  # this is the base link of the robot
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = qw
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            msg.pose_stamped = pose
            msg.timeout = rospy.Time(0.1)
            req.ik_request = msg
            print('before calling service...')
            resp1 = ros_srv(req)

            # del ros_srv

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            sys.exit(1)
        solution = resp1.solution  # RobotState
        # publish to ROS
        self.display_robot_state(solution)
        # input('next...')
        error = resp1.error_code
        return solution

def test_pose_plan(planner: MotionPlanner):
  # use current joint as start
  state = planner.move_group.get_current_state()
  goal = planner.move_group.get_random_pose()
  goal = planner.pose_msg_to_tf(goal)
  plan = planner.pose_motion_plan(state.joint_state, goal, [])
  print(plan)
#   com = input('executing?').strip()
#   if com == 'y':
  planner.execute_plan(plan[1])

def test_save_home(planner: MotionPlanner, fname=None):
    if fname is None:
        planner.save_home_joints()
    else:
        planner.save_home_joints(fname)
        

def test_load_home(planner: MotionPlanner):
    goal_joint = planner.load_home_joints()
    state = planner.move_group.get_current_state()
    plan = planner.joint_motion_plan(state.joint_state, goal_joint, [])
    print(plan)
    # com = input('executing?').strip()
    # if com == 'y':
    planner.execute_plan(plan[1])

def test_load_pose(planner: MotionPlanner, fname):
    goal_joint = planner.load_joints(fname)
    state = planner.move_group.get_current_state()
    planner.display_robot_state(state)
    # input('wait...')
    plan = planner.joint_motion_plan(state.joint_state, goal_joint, [])
    print(plan)
    # com = input('executing?').strip()
    # if com == 'y':
    planner.execute_plan(plan[1])


from manipulation.gripper_test import Gripper
if __name__ == "__main__":
  # test
  rospy.init_node('planner_test')
  planner = MotionPlanner()
  
  test_save_home(planner, 'sense_pose')

#   gripper = Gripper()
#   # watter bottle: 0.12
#   gripper.control(0.85)
#   test_load_pose(planner, 'home_position')
#   input('next...')
#   test_load_pose(planner, 'pre_grasp_pose')
#   test_load_pose(planner, 'grasp_pose')
#   gripper.control(0.05)
#   input('next...')

#   test_load_pose(planner, 'home_position')

#   test_load_pose(planner, 'deposit_pose')
#   gripper.control(0.85)
#   input('next...')

#   test_load_pose(planner, 'home_position')


#   test_save_home(planner, 'deposit_pose')
#   test_pose_plan(planner)
#   input('next...')
#   test_load_home(planner)

  pass