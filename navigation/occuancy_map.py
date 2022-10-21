"""
This implements a wrapper to extract the map data pubilshed by navigation stack at
/locobot/move_base/global_costmap/costmap
or
/locobot/rtabmap/grid_map
NOTE: the extracted topic may be "map", and the robot is in "odom" frame. We need to
use the transformation from "odom" to "map" to get the correct relative frame
NOTE: after getting a plan in the occupancy grid frame. We need to first transform the points
to the base frame. Then transform to the corresponding frame of interest such as the robot.
"""

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import transformations as tf
class OccupancyMap:
  def __init__(self):
    self.topic = '/locobot/move_base/global_costmap/costmap'
    self.extract_map()

  def extract_map(self):
    """
    extract the
    - 2D array representing the map values
    - resolution
    - the base frame from the header (default should be "map")
    - the pose in the frame
    NOTE: we may not want to update the map every second. We can specify some frequency to
    update the map instead
    """
    msg = rospy.wait_for_message(self.topic, OccupancyGrid)
    base_frame = msg.header.frame_id
    resol = msg.info.resolution
    width = msg.info.width
    height = msg.info.height
    pos = [msg.info.origin.position.x,msg.info.origin.position.y,msg.info.origin.position.z]
    ori = [msg.info.origin.orientation.w,msg.info.origin.orientation.x,msg.info.origin.orientation.y,msg.info.origin.orientation.z]
    pose = tf.quaternion_matrix(ori)
    pose[0,3] = pos[0]
    pose[1,3] = pos[1]
    pose[2,3] = pos[2]

    self.base_frame = base_frame
    self.resol = resol
    self.width = width
    self.height = height
    self.pose = pose
    data = msg.info.data
    data = np.array(data).reshape((self.height, self.width))  # might be wrong
    self.data = data

  def obtain_value(self, x, y):
    """
    obtain the occpancy value at (x,y) in the map frame (where the map origin is).
    Assuming (x,y) is in the frame of the occupancy grid
    """
    xi = int(np.floor(x / self.resol))
    yi = int(np.floor(y / self.resol))
    return self.data[xi,yi]