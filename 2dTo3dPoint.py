import numpy as np

# Intrinsic Parameters --> TODO: still need to find these
fx = 5.1885790117450188e+02
fy = 5.1946961112127485e+02
cx = 3.2558244941119034e+02
cy = 2.5373616633400465e+02

K = np.array([[fx,   0,  cx],
            [0,    fy, cy],
            [0,    0,   1]])

#dummy list of points as place holder for the ones we care about
image_points=[[0,0], [240,320], [480,640]]
world_points = []
for point in image_points:
    z = point[2]
    uv = np.array([[point[0]],
                   [point[1]],
                   [1]])
    world_point = np.matmul(np.linalg.inv(K), z * uv)

    # then just append depth from RGBD camera
    #TODO: get depth from RGB camera
    depth = 0 #robot.get_depth(point)

    world_points.append(np.concatenate((world_point,depth), axis=-1))

world_points=np.stack(world_points) #this step is just to make a list of numpy arrays into 1 numpy array