import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import transformations as tf
from matplotlib import pyplot as plt
import math
import matplotlib.pyplot as plt
from heapq import heappush, heappop
import tf2_ros

#from here: https://github.com/richardos/occupancy-grid-a-star
# from occupancy_grid_a_star.a_star import a_star
# from occupancy_grid_a_star.gridmap import OccupancyGridMap
# from occupancy_grid_a_star.utils import plot_path

from navigationConfig import *
''' Code from here: https://github.com/richardos/occupancy-grid-a-star'''
def dist2d(point1, point2):
    """
    Euclidean distance between two points
    :param point1:
    :param point2:
    :return:
    """

    x1, y1 = point1[0:2]
    x2, y2 = point2[0:2]

    dist2 = (x1 - x2)**2 + (y1 - y2)**2

    return math.sqrt(dist2)

def plot_path(path):
    start_x, start_y = path[0]
    goal_x, goal_y = path[-1]

    # plot path
    path_arr = np.array(path)
    plt.plot(path_arr[:, 0], path_arr[:, 1], 'y')

    # plot start point
    plt.plot(start_x, start_y, 'ro')

    # plot goal point
    plt.plot(goal_x, goal_y, 'go')
    plt.savefig('path_test.png')
    # plt.show()

class OccupancyGridMap:
    def __init__(self, data_array, cell_size, occupancy_threshold=0.8):
        """
        Creates a grid map
        :param data_array: a 2D array with a value of occupancy per cell (values from 0 - 1)
        :param cell_size: cell size in meters
        :param occupancy_threshold: A threshold to determine whether a cell is occupied or free.
        A cell is considered occupied if its value >= occupancy_threshold, free otherwise.
        """

        self.data = data_array
        self.dim_cells = data_array.shape
        self.dim_meters = (self.dim_cells[0] * cell_size, self.dim_cells[1] * cell_size)
        self.cell_size = cell_size
        self.occupancy_threshold = occupancy_threshold
        # 2D array to mark visited nodes (in the beginning, no node has been visited)
        self.visited = np.zeros(self.dim_cells, dtype=np.float32)


    def mark_visited_idx(self, point_idx):
        """
        Mark a point as visited.
        :param point_idx: a point (x, y) in data array
        """
        x_index, y_index = point_idx
        # if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
        #     raise Exception('Point is outside map boundary')

        self.visited[y_index][x_index] = 1.0

    def mark_visited(self, point):
        """
        Mark a point as visited.
        :param point: a 2D point (x, y) in meters
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.mark_visited_idx((x_index, y_index))

    def is_visited_idx(self, point_idx):
        """
        Check whether the given point is visited.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is visited, false otherwise
        """
        x_index, y_index = point_idx
        # if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
        #     raise Exception('Point is outside map boundary')

        if self.visited[y_index][x_index] == 1.0:
            return True
        else:
            return False

    def is_visited(self, point):
        """
        Check whether the given point is visited.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is visited, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_visited_idx((x_index, y_index))

    def get_data_idx(self, point_idx):
        """
        Get the occupancy value of the given point.
        :param point_idx: a point (x, y) in data array
        :return: the occupancy value of the given point
        """
        x_index, y_index = point_idx
        # if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
        #     raise Exception('Point is outside map boundary')

        return self.data[y_index,x_index]

    def get_data(self, point):
        """
        Get the occupancy value of the given point.
        :param point: a 2D point (x, y) in meters
        :return: the occupancy value of the given point
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.get_data_idx((x_index, y_index))

    def set_data_idx(self, point_idx, new_value):
        """
        Set the occupancy value of the given point.
        :param point_idx: a point (x, y) in data array
        :param new_value: the new occupancy values
        """
        x_index, y_index = point_idx
        # if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
        #     raise Exception('Point is outside map boundary')

        self.data[y_index][x_index] = new_value

    def set_data(self, point, new_value):
        """
        Set the occupancy value of the given point.
        :param point: a 2D point (x, y) in meters
        :param new_value: the new occupancy value
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        self.set_data_idx((x_index, y_index), new_value)

    def is_inside_idx(self, point_idx):
        """
        Check whether the given point is inside the map.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is inside the map, false otherwise
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            return False
        else:
            return True

    def is_inside(self, point):
        """
        Check whether the given point is inside the map.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is inside the map, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_inside_idx((x_index, y_index))

    def is_occupied_idx(self, point_idx):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is occupied, false otherwise
        """
        x_index, y_index = point_idx
        if self.get_data_idx((x_index, y_index)) >= self.occupancy_threshold:
            return True
        else:
            return False

    def is_occupied(self, point):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is occupied, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_occupied_idx((x_index, y_index))

    def get_index_from_coordinates(self, x, y):
        """
        Get the array indices of the given point.
        :param x: the point's x-coordinate in meters
        :param y: the point's y-coordinate in meters
        :return: the corresponding array indices as a (x, y) tuple
        """
        x_index = int(round(x/self.cell_size))
        y_index = int(round(y/self.cell_size))

        return x_index, y_index

    def get_coordinates_from_index(self, x_index, y_index):
        """
        Get the coordinates of the given array point in meters.
        :param x_index: the point's x index
        :param y_index: the point's y index
        :return: the corresponding point in meters as a (x, y) tuple
        """
        x = x_index*self.cell_size
        y = y_index*self.cell_size

        return x, y

    def plot(self, alpha=1, min_val=0, origin='lower'):
        """
        plot the grid map
        """
        plt.imshow(self.data, vmin=min_val, vmax=1, origin=origin, interpolation='none', alpha=alpha)
        plt.draw()
        plt.savefig('map.png')
        plt.imshow(self.data, vmin=min_val, vmax=1, origin=origin, interpolation='none', alpha=alpha)
        plt.draw()

def _get_movements_4n():
    """
    Get all possible 4-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]

def _get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]

def a_star(start_m, goal_m, gmap, movement='8N', occupancy_cost_factor=3):
    """
    A* for 2D occupancy grid.

    :param start_m: start node (x, y) in meters
    :param goal_m: goal node (x, y) in meters
    :param gmap: the grid map
    :param movement: select between 4-connectivity ('4N') and 8-connectivity ('8N', default)
    :param occupancy_cost_factor: a number the will be multiplied by the occupancy probability
        of a grid map cell to give the additional movement cost to this cell (default: 3).

    :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
    """

    # get array indices of start and goal
    start = gmap.get_index_from_coordinates(start_m[0], start_m[1])
    goal = gmap.get_index_from_coordinates(goal_m[0], goal_m[1])

    # check if start and goal nodes correspond to free spaces
    if gmap.is_occupied_idx(start):
        raise Exception('Start node is not traversable')

    if gmap.is_occupied_idx(goal):
        raise Exception('Goal node is not traversable')

    # add start node to front
    # front is a list of (total estimated cost to goal, total cost from start to node, node, previous node)
    start_node_cost = 0
    start_node_estimated_cost_to_goal = dist2d(start, goal) + start_node_cost
    front = [(start_node_estimated_cost_to_goal, start_node_cost, start, None)]

    # use a dictionary to remember where we came from in order to reconstruct the path later on
    came_from = {}

    # get possible movements
    if movement == '4N':
        movements = _get_movements_4n()
    elif movement == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')

    # while there are elements to investigate in our front.
    while front:
        # get smallest item and remove from front.
        element = heappop(front)

        # if this has been visited already, skip it
        total_cost, cost, pos, previous = element
        if gmap.is_visited_idx(pos):
            continue

        # now it has been visited, mark with cost
        gmap.mark_visited_idx(pos)

        # set its previous node
        came_from[pos] = previous

        # if the goal has been reached, we are done!
        if pos == goal:
            break

        # check all neighbors
        for dx, dy, deltacost in movements:
            # determine new position
            new_x = pos[0] + dx
            new_y = pos[1] + dy
            new_pos = (new_x, new_y)

            # check whether new position is inside the map
            # if not, skip node
            if not gmap.is_inside_idx(new_pos):
                continue

            # add node to front if it was not visited before and is not an obstacle
            if (not gmap.is_visited_idx(new_pos)) and (not gmap.is_occupied_idx(new_pos)):
                potential_function_cost = gmap.get_data_idx(new_pos)*occupancy_cost_factor
                new_cost = cost + deltacost + potential_function_cost
                new_total_cost_to_goal = new_cost + dist2d(new_pos, goal) + potential_function_cost

                heappush(front, (new_total_cost_to_goal, new_cost, new_pos, pos))

    # reconstruct path backwards (only if we reached the goal)
    path = []
    path_idx = []
    if pos == goal:
        while pos:
            path_idx.append(pos)
            # transform array indices to meters
            pos_m_x, pos_m_y = gmap.get_coordinates_from_index(pos[0], pos[1])
            path.append((pos_m_x, pos_m_y))
            pos = came_from[pos]

        # reverse so that path is from start to goal.
        path.reverse()
        path_idx.reverse()

    return path, path_idx
''' end of code from here: https://github.com/richardos/occupancy-grid-a-star'''

from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
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
    # clear the costmap before getting it
    rospy.wait_for_service('/locobot/move_base/clear_costmaps')
    try:
        clear_costmap = rospy.ServiceProxy('/locobot/move_base/clear_costmaps', Empty)
        resp1 = clear_costmap()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    rospy.sleep(1.0)
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
    return data, pose, msg.info.resolution


def planPath(start, end, occupancy_map, resolution):
    # load the map
    #the whole thing is 4 meters, so can divde by the number of grids?; should give value in meters
    print('resolution: ', resolution)
    print(occupancy_map.shape)
    gmap = OccupancyGridMap(occupancy_map, cell_size=resolution, occupancy_threshold=obstacle_map_threshold)

    # set a start and an end node (in meters)
    start_node = (start[0], start[1])
    goal_node = (end[0], end[1])

    gmap.plot()
    
    # run A*
    path, path_px = a_star(start_node, goal_node, gmap, movement='8N')

    

    if path:
        # plot resulting path in pixels over the map
        plot_path(path_px)
        # breakpoint()
        # plt.show()
        return path #[path, path_px]
    else:
        print('Goal is not reachable')
        # breakpoint()
        return None


def followPath(path):
    dist_path = np.diff(path, axis=0)
    path_angles = np.array([np.arctan2(point[0], point[1]) for point in dist_path]).reshape(-1,1)
    new_path = np.hstack([dist_path, path_angles])
    for p in new_path:
        robot_controller.step(p)


def navToPointsFound(people, trash_point, maybes, tfBuffer: tf2_ros.Buffer):
    # obstacle_map, pose, resol = getObstacleMap()
    # breakpoint()
    #robot in world position
    # trans = tfBuffer.lookup_transform('map', 'locobot/base_link', rospy.Time())  # color in base
    # pos = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
    # ori = [trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z]
    # base_in_world = np.eye(4)
    # base_in_world = tf.transformations.quaternion_matrix([ori[0],ori[1],ori[2],ori[3]])
    # base_in_world[0,3] = pos[0]
    # base_in_world[1,3] = pos[1]
    # base_in_world[2,3] = pos[2]

    # offset_base_in_world = np.array(base_in_world)[:2,3] + robot_radius * base_in_world[:2,0]
    # print('offset base in world: ', offset_base_in_world)
    #world to map transformation using pose
    # map_in_world = pose
    # world_in_map = np.linalg.inv(map_in_world)
    
    # base_in_map = world_in_map.dot(base_in_world)
    
    # start = base_in_map[:2,3]+robot_radius * base_in_map[:2,0]
    # print(start)
    trash_with_offset=trash_point[:-1]-trash_pos_offset
    trash_in_base = np.zeros((3))
    trash_in_base[0] = trash_with_offset[0]
    trash_in_base[1] = trash_with_offset[1]
    # trash_in_map = base_in_map[:3,:3].dot(trash_in_base) + base_in_map[:3,3]

    # goal = trash_in_map[:2]
    
    # trash_in_world = base_in_world[:3,:3].dot(trash_in_base) + base_in_world[:3,3]
    # print('start in map: ', start)

    # print('trash in world: ', trash_in_world)
    # print('trash in map: ', goal)
    # goal[0] = min(obstacle_map.shape[0]*resol, goal[0])
    # goal[1] = min(obstacle_map.shape[1]*resol, goal[1])

    # path = planPath(start, goal, obstacle_map, resol)
    # path=np.vstack([base_in_map[:2,3], path])
    pos_diff=trash_point - trash_in_base
    # angle=np.arctan2(pos_diff[1], pos_diff[0])# * 0.9
    angle = np.arctan2(trash_point[1], trash_point[0])
    robot_controller.step([trash_in_base[0], trash_in_base[1], angle])
    # input('beore followpath')
    # if path is not None:
    #     print('path', path)
    #     followPath(path)
    # input('finished')

def continueNavCircuit():
    obstacle_map = getObstacleMap()
    # process obstacle map to get x,y points of the obstacles for the a* algorithm

    #TODO: pick a random empty point from the obstacle map
    end = 0 #random empty point here

    #TODO: plan path to there
    start = 0 #ToDO: get robot start point
    path = planPath(start, end, obstacle_map)
    followPath(path)

# if __name__=='__main__':
