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
  def pick_with_grasp(self):
    pass