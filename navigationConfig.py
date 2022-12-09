from navigation.simple_controller import SimpleController
import numpy as np

robot_controller = SimpleController()

obstacle_map_threshold = 60  # ToDO: play around with this obstacle threshold

trash_pos_offset = np.array([0.45, 0])
robot_radius = 0.5