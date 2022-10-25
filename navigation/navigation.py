from simple_controller import SimpleController
from occupancy_map import OccupancyMap
class Navigation:
  def __init__(self, nav_controller: SimpleController):
    self.controller = nav_controller
    self.map_manager = OccupancyMap()
  def goto(self, x, y, theta):
    """
    given a relative pose to the robot in [x,y,theta], let the robot go to that location.
    Simple version 1: go in straightline to that pose
    TODO: add a planning algorithm to avoid collision
    """
    self.controller.step([x,y,theta])
