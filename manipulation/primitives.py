"""
including:
- pick_with_grasp
- place_with_grasp
- pick_with_obj
- place_with_region
"""

from scripts.manipulation.motion_planner import MotionPlanner

class Primitives:
  def __init__(self, planner: MotionPlanner):
    self.planner = planner
  def pick_with_grasp(self, grasp_joint):
    # * generate pre-grasp pose (retreat for some distance)
    # * motion planning to the pre-grasp pose
    pass
  def place_with_grasp(self, grasp_joint, obj_pose_in_grasp, place_joint):
    # * generate a motion plan from grasp joint to place joint, with object attached
    pass