import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import transformations as tf
from pyhpp.a_star import  AStar

def getObstacleMap():
    """
        extract the
        - 2D array representing the map values
        - resolution
        - the base frame from the header (default should be "map")
        - the pose in the frame
        NOTE: we may not want to update the map every second. We can specify some frequency to
        update the map instead
        """
    msg = rospy.wait_for_message('/locobot/move_base/global_costmap/costmap', OccupancyGrid)
    # base_frame = msg.header.frame_id
    # resol = msg.info.resolution
    width = msg.info.width
    height = msg.info.height
    pos = [msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z]
    ori = [msg.info.origin.orientation.w, msg.info.origin.orientation.x, msg.info.origin.orientation.y,
           msg.info.origin.orientation.z]
    pose = tf.quaternion_matrix(ori)
    pose[0, 3] = pos[0]
    pose[1, 3] = pos[1]
    pose[2, 3] = pos[2]

    data = msg.data
    data = np.array(data).reshape((height, width))  # might be wrong
    return data


# TODO: incorporate obstacle avoidance
def planPath(start, end, obstacle_map):
    assert(obstacle_map.shape[0]==obstacle_map.shape[1])
    # if it's not square, we might have to make it square for this to work

    #x,y,z takes in the coordinates of the obstacles, so we might not even need the obstacle map for this?
    #although I think it should be some amalgamation of both
    scenario = {
        "dimension": {"x": 10, "y": 10, "z": 0},
        "waypoint": {
            "start": {"x": start[0], "y": start[1], "z": 0},
            "stop": {"x": end[0], "y" : end[1], "z": 0},
            "allowDiagonal": False
            },
        "data": {
            "size": obstacle_map.shape[0],
            "x": [4, 5, 6, 7, 4, 5, 6, 7, 4, 5, 6, 7, 4, 5, 6, 7],
            "y": [6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6],
            "z": [0]*len(x)
            },
        "boundary": {
            "zCeil": 6,
            "zFloor": 1
            }
    }
    a_star = AStar(scenario)
    pass

def followPath(path):
    pass

def navToPointsFound(people, trash, maybes):
    obstacle_map=getObstacleMap()
    trash_pos_offset = 0 #TODO: do we need to be offset from the trash at all?
    goals = [np.mean(t,axis=0) for t in trash] #not exactly sure what shape trash is going to have for this to work
    start = 0 #ToDO: get robot start point
    for g in goals:
        planPath(start, g, obstacle_map)
    pass

def continueNavCircuit():
    pass