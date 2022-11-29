import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import transformations as tf
# from pyhpp.a_star import  AStar
from matplotlib import pyplot as plt

#from here: https://github.com/richardos/occupancy-grid-a-star
from occupancy_grid_a_star.a_star import a_star
from occupancy_grid_a_star.gridmap import OccupancyGridMap
from occupancy_grid_a_star.utils import plot_path

from navigationConfig import *

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
    return data, pose


# def OLDplanPath(start, end, obstacle_coords, map_size):
#     # based on this https://pypi.org/project/pyhpp/
#     scenario = {
#         # z=0 to make it a 2d mapping problem
#         "dimension": {"x": map_size[0], "y": map_size[1], "z": 0},
#         "waypoint": {
#             "start": {"x": start[0], "y": start[1], "z": 0},
#             "stop": {"x": end[0], "y" : end[1], "z": 0},
#             "allowDiagonal": False
#             },
#         "data": {
#             "size": len(obstacle_coords[0]),
#             "x": obstacle_coords[0],
#             "y": obstacle_coords[1],
#             },
#         "boundary": {
#             "zCeil": 6,
#             "zFloor": 1
#             }
#     }
#     a_star = AStar(scenario)
#     result = a_star.calculate_path()
#     path = np.stack([result['path']['x'], result['path']['y']]).T
#     return path #returns the int coords of where to go to avoid obstacles and reach goal

def planPath(start, end, occupancy_map):
    # load the map
    #the whole thing is 4 meters, so can divde by the number of grids?; should give value in meters
    gmap = OccupancyGridMap(occupancy_map, cell_size=4/occupancy_map.shape[0], occupancy_threshold=obstacle_map_threshold)

    # set a start and an end node (in meters)
    start_node = tuple(start)
    goal_node = tuple(end)

    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')

    gmap.plot()

    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        plt.show()
        return [path, path_px]
    else:
        print('Goal is not reachable')

        # plot start and goal points over the map (in pixels)
        start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
        goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

        plt.plot(start_node_px[0], start_node_px[1], 'ro')
        plt.plot(goal_node_px[0], goal_node_px[1], 'go')

    plt.show()

def followPath(path):
    #this path is the global position, not relative to the previous position
    # also it's in discrete grid mode (so like image coords), and not in meters
    #TODO:preprocess path so it's in the format we need
    #TODO: also add an angle to it
    for p in path:
        robot_controller.step(p)
    pass

def navToPointsFound(people, trash, maybes):
    trash_pos_offset = 0 #TODO: do we need to be offset from the trash at all?, also move this to config
    goals = [np.mean(t,axis=0) for t in trash] #not exactly sure what shape trash is going to have for this to work

    for g in goals:
        obstacle_map, pose = getObstacleMap()
        # process obstacle map to get x,y points of the obstacles for the a* algorithm
        obstacle_coords = [[], []]
        for i in range(obstacle_map.shape[0]):
            for j in range(obstacle_map.shape[1]):
                if obstacle_map[i,j]> obstacle_map_threshold:
                    obstacle_coords[0].append(i)
                    obstacle_coords[1].append(j)

        start = 0 #ToDO: get robot start point
        path = planPath(start, g, obstacle_coords, obstacle_map.shape)
        followPath(path)


def continueNavCircuit():
    obstacle_map = getObstacleMap()
    # process obstacle map to get x,y points of the obstacles for the a* algorithm
    obstacle_coords = [[], []]
    for i in range(obstacle_map.shape[0]):
        for j in range(obstacle_map.shape[1]):
            if obstacle_map[i, j] > obstacle_map_threshold:
                obstacle_coords[0].append(i)
                obstacle_coords[1].append(j)

    #TODO: pick a random empty point from the obstacle map
    end = 0 #random empty point here

    #TODO: plan path to there
    start = 0 #ToDO: get robot start point
    path = planPath(start, end, obstacle_coords, obstacle_map.shape)
    followPath(path)