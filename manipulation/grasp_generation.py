"""
generate the grasp poses given pcd input using pygpg
TODO: add custom configuration setup when using pygpg
"""
import numpy as np
import pygpg
import transformations as tf
import rospkg
import os
from urdfpy import URDF
import trimesh
from motion_planner import MotionPlanner
import rospy
import open3d as o3d

def grasp_pose_generation_cylinder(obj_pose: np.ndarray, height: float, radius: float, motion_planner: MotionPlanner):
    """
    obj_pose: the pose of the cylinder in the robot frame
    height, radius: specify the shape of the object
    """
    # centered at the object, the grasping pose of the gripper
    gripper_frame_normal = np.array([1,0,0])
    gripper_frame_in_obj = np.eye(4)
    gripper_frame_in_obj[:3,3] = -gripper_frame_normal * radius

    # generate a database of gripper frame in obj
    ang = np.linspace(start=0.0, stop=np.pi*2, num=10)
    height = np.linspace(start=0.0, stop=height, num=10)

    ee_link_name = 'locobot/ee_gripper_link'
    gripper_frame_locals = []
    gripper_frames = []
    for i in range(len(ang)):
        for j in range(len(height)):
            gripper_frame_local = tf.rotation_matrix(ang[i], [0,0,1]).dot(gripper_frame_in_obj)
            gripper_frame_local[2,3] += height[j]
            gripper_frame_locals.append(gripper_frame_local)
            gripper_frame = obj_pose.dot(gripper_frame_local)
            gripper_frames.append(gripper_frame)
    # get robot ik for the grasp pose
    print('before calling get_ik...')
    solution = motion_planner.get_ik(gripper_frames[0], ee_link_name)
    print('solution: ')
    print(solution)

def grasp_pose_generation_at_obj_pose(obj_pose: np.ndarray, obj_pcd: np.ndarray, motion_planner: MotionPlanner):

    # rp = rospkg.RosPack()
    # package_path = rp.get_path('pracsys_ctvmp')
    # urdf = os.path.join(package_path, 'data/models/robot/robotiq_85/robotiq_with_base.urdf')

    # gripper_id = p.loadURDF(urdf, [0,0,0], [0,0,0,1], useFixedBase=False, physicsClientId=robot.pybullet_id)


    # * use gpg to find grasp poses
    base_link_in_grasp1 = np.eye(4)
    # the base link axis are not the same as the grasp frame. Need to coordinate
    base_link_in_grasp1[:3,0] = np.array([0,0,-1])
    base_link_in_grasp1[:3,1] = np.array([0,1,0])
    base_link_in_grasp1[:3,2] = np.array([1,0,0])

    base_link_in_grasp2 = np.eye(4)
    # turning 180 degree
    base_link_in_grasp2[:3,0] = np.array([0,0,1])
    base_link_in_grasp2[:3,1] = np.array([0,-1,0])
    base_link_in_grasp2[:3,2] = np.array([1,0,0])


    grasps = pygpg.generate_grasps(obj_pcd)
    retreat_val = 0.02

    valid_poses_in_obj = []
    valid_joints = []


    for i in range(len(grasps)):
        grasp_bottom = grasps[i].get_grasp_bottom()
        grasp_top = grasps[i].get_grasp_top()
        grasp_surface = grasps[i].get_grasp_surface()
        grasp_approach = grasps[i].get_grasp_approach()
        grasp_binormal = grasps[i].get_grasp_binormal()
        grasp_axis = grasps[i].get_grasp_axis()
        grasp_width = grasps[i].get_grasp_width()
        grasp_sample = grasps[i].get_sample()
        grasp_score = grasps[i].get_score()
        grasp_full_antipodal = grasps[i].is_full_antipodal()
        grasp_half_antipodal = grasps[i].is_half_antipodal()
        print('grasps[%d]: %d' % (i, grasp_half_antipodal))
        if not grasp_half_antipodal and not grasp_full_antipodal:
            continue

        # filter grasp poses which have approach vector pointing up
        z_theta = np.abs(np.pi/2-np.arccos(grasp_approach[2]))
        print('grasp_approach: ', grasp_approach)
        print('z_theta: ', z_theta)
        if grasp_approach[2] > 0 and z_theta >= 30*np.pi/180:
            continue

        grasp_base = grasp_top - grasp_approach * retreat_val

        # check IK for the grasp pose
        grasp_pose = np.eye(4)
        grasp_pose[:3,0] = grasp_approach / np.linalg.norm(grasp_binormal)
        grasp_pose[:3,1] = grasp_binormal / np.linalg.norm(grasp_binormal)
        grasp_pose[:3,2] = grasp_axis / np.linalg.norm(grasp_axis)
        grasp_pose[:3,3] = grasp_base
        grasp_pose1 = grasp_pose.dot(base_link_in_grasp1)
        qw, qx, qy, qz = tf.quaternion_from_matrix(grasp_pose1)

        # TODO: 
        # p.resetBasePositionAndOrientation(gripper_id, grasp_pose1[:3,3], [qx,qy,qz,qw])
        # input('after resetting...')
        # valid1, dof_joint_vals1 = robot.get_ik(robot.tip_link_name[arm], grasp_pose1[:3,3], [qx,qy,qz,qw], 
        #                                      robot.joint_vals, 
        #                                      lower_lim=None, upper_lim=None, jr=None, 
        #                                      collision_check=True, workspace=workspace, 
        #                                      collision_obj_ids=[], visualize=False)
        ee_link_name = 'locobot/ee_gripper_link'
        # pose_in_obj, joints =
        solution = motion_planner.get_ik(grasp_pose1, ee_link_name)
        print('ik solution: ')
        print(solution)

        # valid_poses_in_obj.append(...)
        # valid_joints.append(...)


        grasp_pose2 = grasp_pose.dot(base_link_in_grasp2)
        qw, qx, qy, qz = tf.quaternion_from_matrix(grasp_pose2)

        # p.resetBasePositionAndOrientation(gripper_id, grasp_pose2[:3,3], [qx,qy,qz,qw])
        # input('after resetting...')
        # valid2, dof_joint_vals2 = robot.get_ik(robot.tip_link_name[arm], grasp_pose2[:3,3], [qx,qy,qz,qw], 
        #                                      robot.joint_vals, 
        #                                      lower_lim=None, upper_lim=None, jr=None, 
        #                                      collision_check=True, workspace=workspace, 
        #                                      collision_obj_ids=[], visualize=False)
        solution = motion_planner.get_ik(grasp_pose2, ee_link_name)
        # valid_poses_in_obj.append(...)
        # valid_joints.append(...)


        # if valid1:
        #     pose_in_obj1 = np.linalg.inv(obj_pose).dot(grasp_pose1)
        #     valid_poses_in_obj.append(pose_in_obj1)
        #     valid_joints.append(dof_joint_vals1)
        # if valid2:
        #     pose_in_obj2 = np.linalg.inv(obj_pose).dot(grasp_pose2)
        #     valid_poses_in_obj.append(pose_in_obj2)
        #     valid_joints.append(dof_joint_vals2)

    return valid_poses_in_obj, valid_joints
"""
def grasp_pose_generation_at_obj_poses(obj_poses, obj_pcd: np.ndarray, robot: Robot, arm: str, workspace: Workspace):
    # rp = rospkg.RosPack()
    # package_path = rp.get_path('pracsys_ctvmp')
    # urdf = os.path.join(package_path, 'data/models/robot/robotiq_85/robotiq_with_base.urdf')

    # gripper_id = p.loadURDF(urdf, [0,0,0], [0,0,0,1], useFixedBase=False, physicsClientId=robot.pybullet_id)


    # * use gpg to find grasp poses
    base_link_in_grasp1 = np.eye(4)
    # the base link axis are not the same as the grasp frame. Need to coordinate
    base_link_in_grasp1[:3,0] = np.array([0,0,-1])
    base_link_in_grasp1[:3,1] = np.array([0,1,0])
    base_link_in_grasp1[:3,2] = np.array([1,0,0])

    base_link_in_grasp2 = np.eye(4)
    # turning 180 degree
    base_link_in_grasp2[:3,0] = np.array([0,0,1])
    base_link_in_grasp2[:3,1] = np.array([0,-1,0])
    base_link_in_grasp2[:3,2] = np.array([1,0,0])

    transformed_obj_pcd = obj_poses[0][:3,:3].dot(obj_pcd.T).T + obj_poses[0][:3,3]
    grasps = pygpg.generate_grasps(transformed_obj_pcd)
    retreat_val = 0.02

    valid_poses_in_obj = []
    valid_joints = []


    for i in range(len(grasps)):
        grasp_bottom = grasps[i].get_grasp_bottom()
        grasp_top = grasps[i].get_grasp_top()
        grasp_surface = grasps[i].get_grasp_surface()
        grasp_approach = grasps[i].get_grasp_approach()
        grasp_binormal = grasps[i].get_grasp_binormal()
        grasp_axis = grasps[i].get_grasp_axis()
        grasp_width = grasps[i].get_grasp_width()
        grasp_sample = grasps[i].get_sample()
        grasp_score = grasps[i].get_score()
        grasp_full_antipodal = grasps[i].is_full_antipodal()
        grasp_half_antipodal = grasps[i].is_half_antipodal()
        if not grasp_half_antipodal and not grasp_full_antipodal:
            continue

        # filter grasp poses which have approach vector pointing up
        z_theta = np.abs(np.pi/2-np.arccos(grasp_approach[2]))

        if grasp_approach[2] > 0 and z_theta >= 30*np.pi/180:
            continue

        grasp_base = grasp_top - grasp_approach * retreat_val

        # check IK for the grasp pose
        grasp_pose = np.eye(4)
        grasp_pose[:3,0] = grasp_approach / np.linalg.norm(grasp_binormal)
        grasp_pose[:3,1] = grasp_binormal / np.linalg.norm(grasp_binormal)
        grasp_pose[:3,2] = grasp_axis / np.linalg.norm(grasp_axis)
        grasp_pose[:3,3] = grasp_base
        grasp_pose1 = grasp_pose.dot(base_link_in_grasp1)
        qw, qx, qy, qz = tf.quaternion_from_matrix(grasp_pose1)


        # p.resetBasePositionAndOrientation(gripper_id, grasp_pose1[:3,3], [qx,qy,qz,qw])
        # input('after resetting...')
        valid1, dof_joint_vals1 = robot.get_ik(robot.tip_link_name[arm], grasp_pose1[:3,3], [qx,qy,qz,qw], 
                                             robot.joint_vals, 
                                             lower_lim=None, upper_lim=None, jr=None, 
                                             collision_check=True, workspace=workspace, 
                                             collision_obj_ids=[], visualize=False)


        grasp_pose2 = grasp_pose.dot(base_link_in_grasp2)
        qw, qx, qy, qz = tf.quaternion_from_matrix(grasp_pose2)

        # p.resetBasePositionAndOrientation(gripper_id, grasp_pose2[:3,3], [qx,qy,qz,qw])
        # input('after resetting...')
        valid2, dof_joint_vals2 = robot.get_ik(robot.tip_link_name[arm], grasp_pose2[:3,3], [qx,qy,qz,qw], 
                                             robot.joint_vals, 
                                             lower_lim=None, upper_lim=None, jr=None, 
                                             collision_check=True, workspace=workspace, 
                                             collision_obj_ids=[], visualize=False)

        if valid1:
            pose_in_obj1 = np.linalg.inv(obj_poses[0]).dot(grasp_pose1)
            valid_poses_in_obj.append(pose_in_obj1)
            valid_joints.append(dof_joint_vals1)
        if valid2:
            pose_in_obj2 = np.linalg.inv(obj_poses[0]).dot(grasp_pose2)
            valid_poses_in_obj.append(pose_in_obj2)
            valid_joints.append(dof_joint_vals2)

    # * verify the grasp poses at the given object poses, and filter out invalid ones
    valid_flags = np.ones(len(valid_joints)).astype(bool)
    # for each valid grasp pose, obtain the joint valuels at the object pose
    total_valid_joints = [[valid_joints[i]] for i in range(len(valid_flags))]

    print('valid joints at start pose: %d/%d'%(np.sum(valid_flags),len(valid_flags)))
    for i in range(1,len(obj_poses)):
        valid_joints = []
        for j in range(len(valid_flags)):
            if not valid_flags[j]:
                continue
            # verify at the object pose
            grasp_pose = obj_poses[i].dot(valid_poses_in_obj[j])
            qw, qx, qy, qz = tf.quaternion_from_matrix(grasp_pose)
            valid, dof_joint_vals = robot.get_ik(robot.tip_link_name[arm], grasp_pose[:3,3], [qx,qy,qz,qw], 
                                                robot.joint_vals, 
                                                lower_lim=None, upper_lim=None, jr=None, 
                                                collision_check=True, workspace=workspace, 
                                             collision_obj_ids=[], visualize=False)

            if not valid:
                valid_flags[j] = False
            else:
                total_valid_joints[j].append(dof_joint_vals)
    # filter out invalid ones
    filtered_valid_poses_in_obj = []
    filtered_valid_joints = []
    for i in range(len(valid_flags)):
        if not valid_flags[i]:
            continue
        filtered_valid_poses_in_obj.append(valid_poses_in_obj[i])
        filtered_valid_joints.append(total_valid_joints[i])

    return filtered_valid_poses_in_obj, filtered_valid_joints

"""

if __name__ == '__main__':
  rospy.init_node('planner_test')
  pcd = o3d.io.read_point_cloud("merged_cloud.ply")
  pcd = np.asarray(pcd.points)
#   print('pcds')
#   print(np.asarray(pcd.points))
#   o3d.visualization.draw_geometries([pcd])
  planner = MotionPlanner()
#   grasp_pose_generation_at_obj_pose(None, pcd, planner)

  obj_pose = np.eye(4)
  obj_pose[0,3] = 0.129
  obj_pose[1,3] = 0
  grasp_pose_generation_cylinder(np.eye(4), 0.1, 0.02, planner)